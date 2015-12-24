/*******************************************************************************
 * Copyright (c) 2014, 2015  IBM Corporation and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include "GaussianProcessLDPLMultiModel.hpp"
#include "ArrayUtils.hpp"
#include "SerializeUtils.hpp"

namespace loc{
    /**
     ITUModelFunction
     **/
    ITUModelFunction& ITUModelFunction::distanceOffset(double distanceOffset){
        distanceOffset_ = distanceOffset;
        return *this;
    }
    
    void ITUModelFunction::transformFeature(const Location& stateReceiver, const Location& stateTransmitter, double feats[ndim_]) const{
        
        double distOffsetTmp = distanceOffset_;
        double dist = Location::distance(stateReceiver, stateTransmitter, distOffsetTmp);
        double floorDiff = Location::floorDifference(stateReceiver, stateTransmitter);
        
        feats[0] = -10.0*log10(dist);
        feats[1] = 1.0;

        if(floorDiff<1){
            feats[2] = 0.0;
            feats[3] = 0.0;
        }else{
            feats[2] = -floorDiff;
            feats[3] = -1.0;
        }
    }
    
    double ITUModelFunction::predict(const double parameters[4], const double features[4]) const{
        double ypred = 0;
        for(int i=0; i<ndim_; i++){
            ypred += parameters[i]*features[i];
        }
        ypred = ypred<BeaconConfig::minRssi() ? BeaconConfig::minRssi() : ypred;
        return ypred;
    }
    
    
    template<class Archive>
    void ITUModelFunction::serialize(Archive& ar){
        ar(CEREAL_NVP(distanceOffset_));
    }
    
    template void ITUModelFunction::serialize<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive);
    template void ITUModelFunction::serialize<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive);
    
    
    /**
     Implementation of GaussianProcessLDPLMultiModel
     **/
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::
    bleBeacons(BLEBeacons bleBeacons){
        mBLEBeacons = bleBeacons;
        return *this;
    }
    
    /*
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::kernelFunction(std::shared_ptr<KernelFunction> kernel){
        mKernel = kernel;
        return *this;
    }
    */
    
    template<class Tstate, class Tinput>
    std::vector<std::vector<double>> GaussianProcessLDPLMultiModel<Tstate, Tinput>::fitITUModel(Samples samples){
        std::vector<Sample> samplesAveraged = Sample::mean(Sample::splitSamplesToConsecutiveSamples(samples)); // averaging consecutive samples
        std::cout << "#samplesAveraged = " << samplesAveraged.size() << std::endl;
        
        // construct beacon id to index map
        mBeaconIdIndexMap = BLEBeacon::constructBeaconIdToIndexMap(mBLEBeacons);
        
        // convert samples to X, Y matrices
        size_t n = samplesAveraged.size();
        size_t m = mBeaconIdIndexMap.size();
        static const int ndim = mITUModel.ndim_;
        Eigen::MatrixXd X(n, ndim);
        Eigen::MatrixXd Y(n, m);
        Eigen::MatrixXd Actives(n,m);
        
        bool usesMinRssiObs = false;
        
        for(int i=0; i<n; i++){
            Sample smp = samplesAveraged.at(i);
            Location loc = smp.location();
            Beacons beacons = smp.beacons();
            // convert to X
            X.row(i) << loc.x(), loc.y(), loc.z(), loc.floor();
            // convert to Y
            // initialize rssi values by minRssi
            for(int j=0; j<m; j++){
                Y(i, j) = BeaconConfig::minRssi();
                Actives(i,j) = 0.0;
            }
            // Assign active rssi values to Y matrix.
            for(Beacon b: beacons){
                long id = b.id();
                int index = mBeaconIdIndexMap.at(id);
                Y(i, index) = b.rssi();
                // Active matrix
                if(usesMinRssiObs){
                    Actives(i,index) = 1.0;
                }
                else{
                    if(BeaconConfig::checkInRssiRange(b)){
                        Actives(i,index) = 1.0;
                    }
                }
            }
        }
        
        // Fit ITU mean parameter
        Eigen::VectorXd params0;
        {
            // count the number of active observation values
            int nActive = 0;
            for(int i=0; i<samplesAveraged.size(); i++){
                Beacons bs = samplesAveraged.at(i).beacons();
                for(Beacon b: bs){
                    if(usesMinRssiObs){
                        nActive++;
                    }
                    else if(BeaconConfig::checkInRssiRange(b)){
                        nActive++;
                    }
                }
            }
            // least square estimation
            Eigen::MatrixXd Phi(nActive,ndim);
            Eigen::VectorXd Ytmp(nActive);
            nActive = 0;
            for(int i=0; i<samplesAveraged.size(); i++){
                Sample smp = samplesAveraged.at(i);
                Location loc = smp.location();
                Beacons bs = smp.beacons();
                for(Beacon b: bs){
                    bool bIsActive = false;
                    if(usesMinRssiObs){
                        bIsActive = true;
                    } else if(BeaconConfig::checkInRssiRange(b)){
                        bIsActive = true;
                    }
                    if(bIsActive){
                        double features[ndim];
                        long id = b.id();
                        int index = mBeaconIdIndexMap.at(id);
                        BLEBeacon bleBeacon = mBLEBeacons.at(index);
                        mITUModel.transformFeature(loc, bleBeacon, features);
                        
                        for(int j= 0; j<ndim;j++){
                            Phi(nActive, j) = features[j];
                        }
                        Ytmp(nActive) = b.rssi();
                        nActive++;
                    }
                }
            }
            Eigen::MatrixXd A = (Phi.transpose()*Phi).array();
            Eigen::VectorXd b = Phi.transpose()*Ytmp;
            params0 = A.colPivHouseholderQr().solve(b);
        }
        std::cout << "Initial value of ITU parameters = " << params0 <<std::endl;
        
        // Fit parameters for each BLE beacon
        std::vector<std::vector<double>> ITUParameters(m);
        {
            // Prepare matrices
            std::vector<Eigen::MatrixXd> Xmats(m);
            std::vector<Eigen::VectorXd> Ymats(m);
            Eigen::VectorXd lambdavec = ArrayUtils::vectorToEigenVector(trainParams.lambdas);
            Eigen::VectorXd rhovec = ArrayUtils::vectorToEigenVector(trainParams.rhos);
            
            Eigen::MatrixXd Lambdamat = lambdavec.asDiagonal();
            Eigen::MatrixXd Rhomat = rhovec.asDiagonal();
            
            // convert to feature
            for(int j=0; j<m; j++){
                Eigen::MatrixXd Xmat(n, 4);
                Eigen::VectorXd Ymat(n);
                BLEBeacon bleBeacon = mBLEBeacons.at(j);
                for(int i=0; i<n; i++){
                    double features[ndim];
                    Location loc(X(i,0), X(i,1), X(i,2), X(i,3));
                    mITUModel.transformFeature(loc, bleBeacon, features);
                    for(int k = 0; k<ndim;k++){
                        Xmat(i, k) = features[k];
                    }
                    Ymat(i) = Y(i,j);
                }
                Xmats[j] = Xmat;
                Ymats[j] = Ymat;
            }
            
            // iteration
            Eigen::MatrixXd paramsMatrix(m,ndim);
            Eigen::VectorXd activevec(n);
            // initialize parameters
            for(int j=0; j<m; j++){
                paramsMatrix.row(j) = params0;
            }
            bool wasConverged = false;
            for(int k=0; k<trainParams.maxIteration_; k++){
                // Update parameters for each beacon
                for(int j=0; j<m; j++){
                    Eigen::VectorXd paramsTmp = paramsMatrix.row(j);
                    if(Xmats.at(j).rows()>0){
                        for(int i=0; i<n; i++){
                            double ypred = Xmats.at(j).row(i)*paramsTmp;
                            if(BeaconConfig::minRssi()<ypred){
                                activevec(i)=1.0;
                            }else{
                                activevec(i)=0.0;
                            }
                        }
                        Eigen::MatrixXd A = Xmats.at(j).transpose()*(activevec.asDiagonal())*Xmats.at(j) + Lambdamat;
                        Eigen::VectorXd b = Xmats.at(j).transpose()*(activevec.asDiagonal())*Ymats.at(j) + Lambdamat*params0;
                        paramsTmp = A.colPivHouseholderQr().solve(b);
                    }else{
                        paramsTmp = paramsMatrix.row(j);
                    }
                    paramsMatrix.row(j) = paramsTmp;
                }
                {
                    // Update mean ITU parameters;
                    Eigen::VectorXd paramsMean(ndim);
                    for(int j=0; j<ndim; j++){
                        paramsMean(j) = paramsMatrix.col(j).mean();
                    }
                    Eigen::MatrixXd A = Lambdamat + Rhomat;
                    Eigen::VectorXd b = Lambdamat*paramsMean;
                    Eigen::VectorXd params0new = A.colPivHouseholderQr().solve(b);
                    Eigen::VectorXd diff = params0-params0new;
                    params0 = params0new;
                    //std::cout << "params0=" << params0;
                    //std::cout << ", diff.norm()=" << diff.norm() << std::endl;
                    if(diff.norm() < trainParams.tolranceOptimization_){
                        //std::cout << "Optimization loop is finished." << std::endl;
                        wasConverged = true;
                        break;
                    }
                }
            }
            if(!wasConverged){
                std::cout << "ITU parameters were not converged." << std::endl;
            }
            std::cout << "mean(paramseters) = " << params0 << std::endl;
            std::cout << "parameters = " << paramsMatrix << std::endl;
            // Update ITUParameters by optimized parameters
            {
                for(int i=0; i<m; i++){
                    std::vector<double> params(ndim);
                    for( int j=0; j<ndim; j++){
                        params[j] = paramsMatrix(i, j);
                    }
                    ITUParameters[i] = params;
                }
            }
        }

        return ITUParameters;
    }
    
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::train(Samples samples){
        std::vector<Sample> samplesAveraged = Sample::mean(Sample::splitSamplesToConsecutiveSamples(samples)); // averaging consecutive samples
        std::cout << "#samplesAveraged = " << samplesAveraged.size() << std::endl;
        
        // construct beacon id to index map
        assert(mBLEBeacons.size()>0);
        mBeaconIdIndexMap = BLEBeacon::constructBeaconIdToIndexMap(mBLEBeacons);
        
        // convert samples to X, Y matrices
        size_t n = samplesAveraged.size();
        size_t m = mBeaconIdIndexMap.size();
        static const int ndim = mITUModel.ndim_;
        Eigen::MatrixXd X(n, ndim);
        Eigen::MatrixXd Y(n, m);
        Eigen::MatrixXd Actives(n,m);
        
        bool usesMinRssiObs = true;
        
        // FIT ITU model parameters
        mITUParameters = fitITUModel(samples);
        
        for(int i=0; i<n; i++){
            Sample smp = samplesAveraged.at(i);
            Location loc = smp.location();
            Beacons beacons = smp.beacons();
            // convert to X
            X.row(i) << loc.x(), loc.y(), loc.z(), loc.floor();
            // convert to Y
            // initialize rssi values by minRssi
            for(int j=0; j<m; j++){
                Y(i, j) = BeaconConfig::minRssi();
                Actives(i,j) = 0.0;
            }
            // Assign active rssi values to Y matrix.
            for(Beacon b: beacons){
                long id = b.id();
                int index = mBeaconIdIndexMap.at(id);
                Y(i, index) = b.rssi();
                // Active matrix
                if(usesMinRssiObs){
                    Actives(i,index) = 1.0;
                }
                else{
                    if(BeaconConfig::checkInRssiRange(b)){
                        Actives(i,index) = 1.0;
                    }
                }
            }
        }
        
        // Compute dY = Y - m(X)
        Eigen::MatrixXd dY(n, m);
        for(int i=0; i<n; i++){
            Sample smp = samplesAveraged.at(i);
            Location loc = smp.location();
            for(int j=0; j<m; j++){
                BLEBeacon bleBeacon = mBLEBeacons.at(j);
                double features[ndim];
                mITUModel.transformFeature(loc, bleBeacon, features);
                std::vector<double> params = mITUParameters.at(j);
                double ymean = mITUModel.predict(params.data(), features);
                dY(i, j)=Y(i,j)-ymean;
            }
        }
        
        //for(int j=0; j<m; j++){
//            for(int i=0; i<n; i++){
//                std::cout <<  "(Y, dY)=" << Y(i,0) <<"," << dY(i,0)  << std::endl;
//            }
        //}
        
        // Training with selection of kernel parameters
        mGP.fitCV(X, dY, Actives);
        
        // Estimate variance parameter (sigma_n) by using raw (=not averaged) data
        mRssiStandardDeviations = computeRssiStandardDeviations(samples);
        for(int i=0; i<mRssiStandardDeviations.size(); i++){
            std::cout << "stdev(" << i<< ")=" << mRssiStandardDeviations.at(i) <<std::endl;
        }
        mNormalRssiStandardDeviation = computeNormalStandardDeviation(mRssiStandardDeviations);
        
        return *this;
    }
    
    template<class Tstate, class Tinput>
    std::vector<double> GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeRssiStandardDeviations(Samples samples){
        
        std::map<int, int> indexCount;
        std::map<int, double> indexRssiSum;
        
        for(auto iter=mBeaconIdIndexMap.begin(); iter!=mBeaconIdIndexMap.end(); iter++){
            int index = iter->second;
            indexCount[index] = 0;
            indexRssiSum[index] = 0;
        }
        
        for(Sample smp: samples){
            Location loc = smp.location();
            Beacons bs = smp.beacons();
            
            double xvec[4];
            MLAdapter::locationToVec(loc, xvec);
            
            std::vector<int> indices;
            for(Beacon b: bs){
                long id = b.id();
                int index = mBeaconIdIndexMap.at(id);
                indices.push_back(index);
            }
            
            std::vector<double> dypreds = mGP.predict(xvec, indices);
            
            int i = 0;
            for(Beacon b: bs){
                long id = b.id();
                int index = mBeaconIdIndexMap.at(id);
                BLEBeacon ble = mBLEBeacons.at(index);
                double features[4];
                mITUModel.transformFeature(loc, ble, features);
                
                const double* params = mITUParameters.at(index).data();
                double mean = mITUModel.predict(params, features);
                
                double dypred = dypreds.at(i);
                double ypred = mean + dypred;
                double rssi = b.rssi();
                double difference = rssi - ypred;
                
                indexCount[index] += 1;
                indexRssiSum[index] += difference*difference;
                
                i++;
            }
            
        }
        
        std::vector<double> stdevs;
        for(auto iter=mBeaconIdIndexMap.begin(); iter!=mBeaconIdIndexMap.end(); iter++){
            int index = iter->second;
            double var = indexRssiSum[index] /(indexCount[index]);
            double stdev = sqrt(var);
            stdevs.push_back(stdev);
        }
        return stdevs;
    }
    
    template<class Tstate, class Tinput>
    double GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeNormalStandardDeviation(std::vector<double> standardDeviations){
        double sq = 0;
        for(double stdev: standardDeviations){
            sq += stdev*stdev;
        }
        sq/=(standardDeviations.size());
        return std::sqrt(sq);
    }
    
    
    template<class Tstate, class Tinput>
    Tinput GaussianProcessLDPLMultiModel<Tstate, Tinput>::convertInput(const Tinput& input){
        Tinput inputConverted;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            long id = iter->id();
            if(mBeaconIdIndexMap.count(id)==1){
                inputConverted.push_back(*iter);
            }
        }
        return inputConverted;
    }
    
    template<class Tstate, class Tinput>
    double GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeLogLikelihood(const Tstate& state, const Tinput& input){
        //Assuming Tinput = Beacons
        std::vector<int> indices;
        //indices.clear();
        
        int countKnown, countUnknown = 0;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            Beacon b = *iter;
            assert(BeaconConfig::checkInRssiRange(b));
            long id = b.id();
            if(mBeaconIdIndexMap.count(id)==1){
                assert( mBeaconIdIndexMap.count(id)==1 );
                int index = mBeaconIdIndexMap.at(id);
                indices.push_back(index);
            }else{
                countUnknown++;
            }
        }
        countKnown = static_cast<int>(indices.size());
        
        double xvec[4];
        MLAdapter::locationToVec(state, xvec);
        std::vector<double> dypreds = mGP.predict(xvec, indices);
        
        double jointLogLL = 0;
        int i=0;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            Beacon b = *iter;
            double rssi = b.rssi();
            rssi = rssi - state.rssiBias();
            long id = b.id();
            
            // RSSI of known beacons are predicted by a model.
            if(mBeaconIdIndexMap.count(id)==1){
                int index = mBeaconIdIndexMap.at(id);
                BLEBeacon bleBeacon = mBLEBeacons.at(index);
                
                double features[4];
                mITUModel.transformFeature(state, bleBeacon, features);
                const double* params = mITUParameters.at(index).data();
                double mean = mITUModel.predict(params, features);
                double dypred = dypreds.at(i);
                double ypred = mean + dypred;
                double stdev = mRssiStandardDeviations[index];
                
                double logLL = MathUtils::logProbaNormal(rssi, ypred, stdev);
                jointLogLL += logLL;
                i++;
            }
            // RSSI of unknown beacons are assumed to be minRssi.
            else{
                double ypred = BeaconConfig::minRssi();
                double stdev = mNormalRssiStandardDeviation;
                double logLL = MathUtils::logProbaNormal(rssi, ypred, stdev);
                jointLogLL += logLL;
            }
        }
        return jointLogLL;
    }
    
    template<class Tstate, class Tinput>
    std::vector<double> GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeLogLikelihood(const std::vector<Tstate> & states, const Tinput & input) {
        int n = (int) states.size();
        std::vector<double> logLLs(n);
        for(int i=0; i<n; i++){
            logLLs[i] = this->computeLogLikelihood(states.at(i), input);
        }
        return logLLs;
    }
    
    
    // CEREAL function
    template<class Tstate, class Tinput>
    template<class Archive>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(Archive& ar) const{
        //ar(CEREAL_NVP(mKernel));
        ar(CEREAL_NVP(mBLEBeacons));
        ar(CEREAL_NVP(mITUModel));
        ar(CEREAL_NVP(mITUParameters));
        
        ar(CEREAL_NVP(mGP));
        ar(CEREAL_NVP(mRssiStandardDeviations));
    }
    
    template<class Tstate, class Tinput>
    template<class Archive>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(Archive& ar){
        //ar(CEREAL_NVP(mKernel));
        ar(CEREAL_NVP(mBLEBeacons));
        ar(CEREAL_NVP(mITUModel));
        ar(CEREAL_NVP(mITUParameters));
        
        ar(CEREAL_NVP(mGP));
        ar(CEREAL_NVP(mRssiStandardDeviations));
        mBeaconIdIndexMap = BLEBeacon::constructBeaconIdToIndexMap(mBLEBeacons);
        mNormalRssiStandardDeviation = computeNormalStandardDeviation(mRssiStandardDeviations);
    }
    
    //explicit instantiation
    template void GaussianProcessLDPLMultiModel<State, Beacons>::load<cereal::JSONInputArchive>(cereal::JSONInputArchive& archive);
    template void GaussianProcessLDPLMultiModel<State, Beacons>::save<cereal::JSONOutputArchive>(cereal::JSONOutputArchive& archive) const;
    
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(std::ofstream& ofs) const{
        cereal::JSONOutputArchive oarchive(ofs);
        oarchive(*this);
    }
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(std::ifstream& ifs){
        cereal::JSONInputArchive iarchive(ifs);
        iarchive(*this);
    }
    
    
    /**
     Implementation of GaussianProcessLDPLMultiModelTrainer
     **/
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>* GaussianProcessLDPLMultiModelTrainer<Tstate, Tinput>::train(){
        
        Samples samples = mDataStore->getSamples();
        BLEBeacons bleBeacons = mDataStore->getBLEBeacons();
        
        assert(samples.size()>0);
        assert(bleBeacons.size()>0);
        
        GaussianProcessLDPLMultiModel<Tstate, Tinput>* obsModel = new GaussianProcessLDPLMultiModel<Tstate, Tinput>();
        obsModel->bleBeacons(bleBeacons);
        obsModel->train(samples);
        
        return obsModel;
    }
    
    
    //Explicit instantiation
    template class GaussianProcessLDPLMultiModel<State, Beacons>;
    template class GaussianProcessLDPLMultiModelTrainer<State, Beacons>;
    
}