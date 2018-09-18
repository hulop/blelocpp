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
#include "DataLogger.hpp"

#include "GaussianProcessLight.hpp"

//#include "ExtendedDataUtils.hpp"

namespace loc{
    /**
     ITUModelFunction
     **/
    ITUModelFunction& ITUModelFunction::distanceOffset(double distanceOffset){
        distanceOffset_ = distanceOffset;
        return *this;
    }
    
    /*
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
    */
    
    std::vector<double> ITUModelFunction::transformFeature(const Location& stateReceiver, const Location& stateTransmitter) const{
        std::vector<double> feats(ndim_);
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
        return feats;
    }
    
    double ITUModelFunction::predict(const double *parameters, const double *features) const{
        double ypred = 0;
        for(int i=0; i<ndim_; i++){
            ypred += parameters[i]*features[i];
        }
        ypred = ypred<BeaconConfig::minRssi() ? BeaconConfig::minRssi() : ypred;
        return ypred;
    }
    
    double ITUModelFunction::predict(const std::vector<double>& parameters, const std::vector<double>& features) const{
        return predict(parameters.data(), features.data());
    }
    
    
    template<class Archive>
    void ITUModelFunction::serialize(Archive& ar){
        ar(CEREAL_NVP(distanceOffset_));
    }
    
    template void ITUModelFunction::serialize<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive);
    template void ITUModelFunction::serialize<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive);
    template void ITUModelFunction::serialize<cereal::PortableBinaryInputArchive> (cereal::PortableBinaryInputArchive& archive);
    template void ITUModelFunction::serialize<cereal::PortableBinaryOutputArchive> (cereal::PortableBinaryOutputArchive& archive);
    
    
    /**
     Implementation of GaussianProcessLDPLMultiModel
     **/
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::
    bleBeacons(BLEBeacons bleBeacons){
        mBLEBeacons = bleBeacons;
        // construct beacon id to index map
        mBeaconIdIndexMap = BLEBeacon::constructBeaconIdToIndexMap(mBLEBeacons);
        
        for(const auto& bleBeacon:mBLEBeacons){
            const auto& id = bleBeacon.id();
            mITUModelMap[id] = ITUModelFunction();
        }
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
        if(samplesAveraged.size()==0){
            throw std::runtime_error("No valid sample [samplesAveraged.size()==0]");
        }
        
        // convert samples to X, Y matrices
        size_t n = samplesAveraged.size();
        size_t m = mBeaconIdIndexMap.size();
        static const int ndim = ITUModelFunction::ndim_;
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
                const auto& id = b.id();
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
                        const auto& id = b.id();
                        int index = mBeaconIdIndexMap.at(id);
                        BLEBeacon bleBeacon = mBLEBeacons.at(index);
                        auto features = mITUModelMap[id].transformFeature(loc, bleBeacon);
                        
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
        std::cout << "Initial value of ITU parameters = " << params0.transpose() <<std::endl;
        
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
                Eigen::MatrixXd Xmat(n, ndim);
                Eigen::VectorXd Ymat(n);
                BLEBeacon bleBeacon = mBLEBeacons.at(j);
                for(int i=0; i<n; i++){
                    const auto& id = bleBeacon.id();
                    Location loc(X(i,0), X(i,1), X(i,2), X(i,3));
                    auto features = mITUModelMap[id].transformFeature(loc, bleBeacon);
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
            std::cout << "mean(parameters) = " << params0.transpose() << std::endl;
            //std::cout << "parameters = " << paramsMatrix << std::endl;
            for(auto & ble: mBLEBeacons){
                int index = mBeaconIdIndexMap.at(ble.id());
                std::cout << "parameters(" << ble.major() << "," << ble.minor() << ") = " <<paramsMatrix.row(index) << std::endl;
            }
            
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
        
        if(gpType==GPNORMAL){
            mGP = std::make_shared<GaussianProcess>();
        }else{
            mGP = std::make_shared<GaussianProcessLight>();
        }
        
        if(matType==DENSE){
            mGP->setAsSparse(false);
        }else if(matType==SPARSE){
            mGP->setAsSparse(true);
        }
        
        std::vector<Sample> samplesAveraged = Sample::mean(Sample::splitSamplesToConsecutiveSamples(samples)); // averaging consecutive samples
        std::cout << "#samplesAveraged = " << samplesAveraged.size() << std::endl;
        
        // construct beacon id to index map
        if(mBLEBeacons.size() <= 0){
            BOOST_THROW_EXCEPTION(LocException("BLEBeacons have not been set to this instance."));
        }
        
        // convert samples to X, Y matrices
        size_t n = samplesAveraged.size();
        size_t m = mBeaconIdIndexMap.size();
        static const int ndim = ITUModelFunction::ndim_;
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
                const auto& id = b.id();
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
                const auto& id = bleBeacon.id();
                auto features = mITUModelMap[id].transformFeature(loc, bleBeacon);
                std::vector<double> params = mITUParameters.at(j);
                double ymean = mITUModelMap[id].predict(params, features);
                dY(i, j)=Y(i,j)-ymean;
            }
        }
        
        // Training with selection of kernel parameters
        mGP->fitCV(X, dY, Actives);
        
        // Estimate variance parameter (sigma_n) by using raw (=not averaged) data
        mRssiStandardDeviations = computeRssiStandardDeviations(samples);
        for(auto& ble: mBLEBeacons){
            const auto& id = ble.id();
            int index = mBeaconIdIndexMap.at(id);
            std::cout << "stdev(" <<ble.major() << "," << ble.minor() << ") = " << mRssiStandardDeviations.at(index) <<std::endl;
        }
        
        if(mStdevRssiForUnknownBeacon==0){
            mStdevRssiForUnknownBeacon = computeNormalStandardDeviation(mRssiStandardDeviations);
        }
        
        return *this;
    }
    
    // compute standard deviation of RSSI for each ble beacon
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
            
            std::vector<double> xvec = MLAdapter::locationToVec(loc);
            std::vector<int> indices = extractKnownBeaconIndices(bs);
            std::vector<double> dypreds = mGP->predict(xvec.data(), indices);
            
            int i = 0;
            for(const Beacon& b: bs){
                const auto& id = b.id();
                int index = mBeaconIdIndexMap.at(id);
                BLEBeacon ble = mBLEBeacons.at(index);
                std::vector<double> features = mITUModelMap[id].transformFeature(loc, ble);
                std::vector<double> params = mITUParameters.at(index);
                double mean = mITUModelMap[id].predict(params, features);
                
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
        for(auto& ble: mBLEBeacons){
            const auto& id = ble.id();
            int index = mBeaconIdIndexMap.at(id);
            double var = indexRssiSum[index] /(indexCount[index]);
            if (isnan(var)) {
                std::cerr << "Stdev is NaN for beacon(" << ble.id().toString() << ")" << std::endl;
            }
            double stdev = sqrt(var);
            stdevs.push_back(stdev);
        }

        return stdevs;
    }
    
    template<class Tstate, class Tinput>
    double GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeNormalStandardDeviation(std::vector<double> standardDeviations){
        double sq = 0;
        int countValid = 0;
        for(double stdev: standardDeviations){
            if(!std::isnan(stdev)){
                sq += stdev*stdev;
                countValid+=1;
            }
        }
        sq/=(countValid);
        return std::sqrt(sq);
    }
    
    
    template<class Tstate, class Tinput>
    Tinput GaussianProcessLDPLMultiModel<Tstate, Tinput>::convertInput(const Tinput& input){
        Tinput inputConverted;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            const auto& id = iter->id();
            if(mBeaconIdIndexMap.count(id)==1){
                inputConverted.push_back(*iter);
            }
        }
        return inputConverted;
    }
    
    template<class Tstate, class Tinput>
    std::vector<int> GaussianProcessLDPLMultiModel<Tstate, Tinput>::extractKnownBeaconIndices(const Tinput& input) const{
        std::vector<int> indices;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            Beacon b = *iter;
            const auto& id = b.id();
            if(mBeaconIdIndexMap.count(id)==1){
                int index = mBeaconIdIndexMap.at(id);
                indices.push_back(index);
            }
        }
        return indices;
    }
    
    template<class Tstate, class Tinput>
    std::map<BeaconId, NormalParameter>  GaussianProcessLDPLMultiModel<Tstate, Tinput>::predict(const Tstate& state, const Tinput& input) const{
        //Assuming Tinput = Beacons
        std::map<BeaconId, NormalParameter> beaconIdRssiStatsMap;
        
        std::vector<double> xvec = MLAdapter::locationToVec(state);
        std::vector<int> indices = extractKnownBeaconIndices(input);
        std::vector<double> dypreds = mGP->predict(xvec.data(), indices);
        
        int idx_local=0;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            auto b = *iter;
            const auto& id = b.id();
            // RSSI of known beacons are predicted by a model.
            if(mBeaconIdIndexMap.count(id)==1){
                int idx_global = mBeaconIdIndexMap.at(id);
                const BLEBeacon& bleBeacon = mBLEBeacons.at(idx_global);
                
                const auto& ituModel = mITUModelMap.at(id);
                const auto& features = ituModel.transformFeature(state, bleBeacon);
                const auto& params = mITUParameters.at(idx_global);
                double mean = ituModel.predict(params, features);
                double dypred = dypreds.at(idx_local);
                
                double ypred = mean + dypred;
                double stdev = mRssiStandardDeviations[idx_global];
                
                if(mCoeffDiffFloorStdev!=1.0 && Location::checkDifferentFloor(state, bleBeacon)){
                    stdev = stdev*mCoeffDiffFloorStdev ;
                }
                
                NormalParameter rssiStats(ypred, stdev);
                beaconIdRssiStatsMap[id] = rssiStats;
                
                idx_local++;
            }
            
        }
        return beaconIdRssiStatsMap;
    }
    
    
    template<class Tstate, class Tinput>
    double GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeLogLikelihood(const Tstate& state, const Tinput& input){
        std::vector<double> values = this->computeLogLikelihoodRelatedValues(state, input);
        return values.at(0);
    }
    
    template<class Tstate, class Tinput>
    std::vector<double> GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeLogLikelihoodRelatedValues(const Tstate& state, const Tinput& input){
        //Assuming Tinput = Beacons
        
        std::vector<double> returnValues(4); // logLikelihood, mahalanobisDistance, #knownBeacons, #unknownBeacons
        
        std::map<BeaconId, NormalParameter> beaconIdRssiStatsMap;
        // delayed prdiction
        int T = mTDelay;
        double dTmin = mDTDelay - mDTDelayMargin; //ms
        double dTmax = mDTDelay + mDTDelayMargin; //ms
        if(T==1){
            beaconIdRssiStatsMap = this->predict(state, input);
        }else{
            long headTS = state.timestamp;
            std::vector<State> statesConsider;
            auto hsize = state.history.size();
            for(int i=0; i<hsize; i++){
                const State& hState = state.history.at(hsize-i-1);
                long diffTS = headTS - hState.timestamp;
                if( dTmin <= diffTS && diffTS<dTmax){
                    headTS = hState.timestamp;
                    statesConsider.push_back(hState);
                    if((T-1)<= statesConsider.size()){
                        break;
                    }
                }
            }
            
            auto nConsidered = statesConsider.size() + 1;
            auto cMap = this->predict(state, input);
            std::map<BeaconId, double> meanPreds;
            double avgWeight = 1.0/((double) nConsidered);
            
            for(auto iter = cMap.begin(); iter!= cMap.end() ; ++iter ) {
                const auto& id = iter->first;
                meanPreds[id] = avgWeight * iter->second.mean();
            }
            for(int i=0; i<statesConsider.size(); i++){
                const State& s = statesConsider.at(i);
                auto tmpBMap = this->predict(s, input);
                for(auto iter = tmpBMap.begin(); iter!= tmpBMap.end() ; ++iter ) {
                    const auto& id = iter->first;
                    meanPreds[id] += avgWeight * iter->second.mean();
                }
            }
            
            std::map<BeaconId, NormalParameter> meanStatsMap;
            for(auto iter = cMap.begin(); iter!= cMap.end() ; ++iter ) {
                const auto& id = iter->first;
                NormalParameter np(meanPreds[id], iter->second.stdev());
                meanStatsMap[id] = np;
            }
            beaconIdRssiStatsMap = meanStatsMap;
        }
        
        std::vector<int> indices = extractKnownBeaconIndices(input);
        
        size_t countKnown = indices.size();
        size_t countUnknown = input.size() - indices.size();

        if(countKnown==0){
            std::cout << "ObservationModel does not know the input data." << std::endl;
        }
        
        double jointLogLL = 0;
        double sumMahaDist = 0;
        for(auto iter=input.begin(); iter!=input.end(); iter++){
            const Beacon& b = *iter;
            double rssi = b.rssi();
            
            const State* pState = dynamic_cast<const State*>(&state);
            if(pState){
                double rssiBias = pState->rssiBias();
                rssi = rssi - rssiBias;
            }
            const auto& id = b.id();
            
            // RSSI of known beacons are predicted by a model.
            if(mBeaconIdIndexMap.count(id)==1){
                auto rssiStats = beaconIdRssiStatsMap[id];
                double ypred = rssiStats.mean();
                double stdev = rssiStats.stdev();
                
                //double logLL = MathUtils::logProbaNormal(rssi, ypred, stdev);
                double logLL = normFunc(rssi, ypred, stdev);
                double mahaDist = MathUtils::mahalanobisDistance(rssi, ypred, stdev);
                
                if(applyLowestLogLikelihood){
                    double enlargedStdev = mStdevRssiForUnknownBeacon * mCoeffDiffFloorStdev;
                    int idx = mBeaconIdIndexMap[b.id()];
                    auto ble = mBLEBeacons.at(idx);
                    if(ble.floor()!=state.floor()){
                        double lowestlogLL = normFunc(0, 0, enlargedStdev);
                        logLL = lowestlogLL < logLL? logLL : lowestlogLL;
                    }
                }

                jointLogLL += logLL;
                sumMahaDist += mahaDist;
                
            }
            // RSSI of unknown beacons are assumed to be minRssi.
            else if(mFillsUnknownBeaconRssi){
                double ypred = BeaconConfig::minRssi();
                double stdev = mStdevRssiForUnknownBeacon;
                
                //double logLL = MathUtils::logProbaNormal(rssi, ypred, stdev);
                double logLL = normFunc(rssi, ypred, stdev);
                double mahaDist = MathUtils::mahalanobisDistance(rssi, ypred, stdev);
                
                jointLogLL += logLL;
                sumMahaDist += mahaDist;
            }
        }
        returnValues[0] = jointLogLL;
        returnValues[1] = sumMahaDist;
        returnValues[2] = countKnown;
        returnValues[3] = countUnknown;
        
        return returnValues;
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
    
    template<class Tstate, class Tinput>
    std::vector<std::vector<double>> GaussianProcessLDPLMultiModel<Tstate, Tinput>::computeLogLikelihoodRelatedValues(const std::vector<Tstate> & states, const Tinput & input) {
        int n = (int) states.size();
        std::vector<double> logLLs(n);
        
        std::vector<std::vector<double>> values(n);
        for(int i=0; i<n; i++){
            values[i] = this->computeLogLikelihoodRelatedValues(states.at(i), input);
        }
        return values;
    }
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::fillsUnknownBeaconRssi(bool fills){
        mFillsUnknownBeaconRssi = fills;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    bool GaussianProcessLDPLMultiModel<Tstate, Tinput>::fillsUnknownBeaconRssi() const{
        return mFillsUnknownBeaconRssi;
    }
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::rssiStandardDeviationForUnknownBeacons(double stdevRssi){
        mStdevRssiForUnknownBeacon = stdevRssi;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    double GaussianProcessLDPLMultiModel<Tstate, Tinput>::rssiStandardDeviationForUnknownBeacons() const{
        return mStdevRssiForUnknownBeacon;
    }
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::coeffDiffFloorStdev(double coeff){
        mCoeffDiffFloorStdev = coeff;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>& GaussianProcessLDPLMultiModel<Tstate, Tinput>::tDelay(int T){
        mTDelay = T;
        State::history_capacity = T;
        return *this;
    }
    
    // CEREAL function
    template<class Tstate, class Tinput>
    template<class Archive>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(Archive& ar) const{
        // in order archive for binary support
        // version 3 (since v1.3.4) : binary support
        std::cout << "saving version = " << version << std::endl;
        ar(CEREAL_NVP(version));
        ar(CEREAL_NVP(mBLEBeacons));
        
        // mITUModelMap
        if(version<=2){
            // key: long, value: ITUModelFunction
            std::map<uint64_t, ITUModelFunction> ITUModelMapV2;
            for(auto iter=mITUModelMap.begin(); iter!=mITUModelMap.end(); iter++){
                auto id = iter->first;
                auto value = iter->second;
                long long_id = BeaconId::convertToLongId(id);
                ITUModelMapV2[long_id] = value;
            }
            ar(cereal::make_nvp("mITUModelMap", ITUModelMapV2));
        }else if(version<=3){
            // key: BeaconId, value: ITUModelFunction
            ar(CEREAL_NVP(mITUModelMap));
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported version (version=" + std::to_string(version) +")"));
        }

        ar(CEREAL_NVP(mITUParameters));

        if(version <= 1){
            ar(cereal::make_nvp("mGP",*mGP));
        }else if(version <= 2){
            auto lgp = std::dynamic_pointer_cast<GaussianProcessLight>(mGP);
            if(lgp){
                ar(cereal::make_nvp("GaussianProcessLight", *lgp));
            }else{
                ar(cereal::make_nvp("GaussianProcess", *mGP));
            }
        }else if(3 <= version){
            // save model type first
            ar(cereal::make_nvp("ModelType", gpType));
            if(gpType == GPType::GPLIGHT){
                auto lgp = std::dynamic_pointer_cast<GaussianProcessLight>(mGP);
                ar(cereal::make_nvp("GaussianProcessLight", *lgp));
            } else if (gpType == GPType::GPNORMAL) {
                ar(cereal::make_nvp("GaussianProcess", *mGP));
            }
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported version (version=" + std::to_string(version) +")"));
        }
        ar(CEREAL_NVP(mRssiStandardDeviations));
                 
        ar(CEREAL_NVP(mTDelay));
    }
    
    template<class Tstate, class Tinput>
    template<class Archive>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(Archive& ar){
        // in order archive for binary support
        
        try{
            ar(CEREAL_NVP(version));
        }catch(cereal::Exception& e){
            version = 0;
        }
        std::cout << "loading version = " << version << std::endl;
        ar(CEREAL_NVP(mBLEBeacons));
        
        // mITUModelMap
        if(version==0){
            ITUModelFunction mITUModel;
            ar(CEREAL_NVP(mITUModel));
            for(const auto& bleBeacon:mBLEBeacons){
                const auto& id = bleBeacon.id();
                mITUModelMap[id] = mITUModel;
            }
        }else if(version<=2){
            // key: long, value: ITUModelFunction
            std::map<uint64_t, ITUModelFunction> ITUModelMapV2;
            ar(cereal::make_nvp("mITUModelMap", ITUModelMapV2));
            for(auto iter = ITUModelMapV2.begin(); iter!=ITUModelMapV2.end(); iter++){
                auto key = iter->first;
                auto value = iter->second;
                const auto& id = BeaconId::convertLongIdToId(key);
                mITUModelMap[id] = value;
            }
        }else if(version<=3){
            // key: BeaconId, value: ITUModelFunction
            ar(CEREAL_NVP(mITUModelMap));
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported version (version=" + std::to_string(version) +")"));
        }
            
        ar(CEREAL_NVP(mITUParameters));
        
        // deserialize gp
        if(version <= 1){
            GaussianProcess gp;
            ar(cereal::make_nvp("mGP", gp));
            this->mGP = std::make_shared<GaussianProcess>(gp);
            this->gpType = GPType::GPNORMAL;
        }else if (version <= 2){
            try{
                GaussianProcessLight lgp;
                ar(cereal::make_nvp("GaussianProcessLight", lgp));
                this->mGP = std::make_shared<GaussianProcessLight>(lgp);
                this->gpType = GPType::GPLIGHT;
            }catch(cereal::Exception& e){
                GaussianProcess gp;
                ar(cereal::make_nvp("GaussianProcess", gp));
                this->mGP = std::make_shared<GaussianProcess>(gp);
                this->gpType = GPType::GPNORMAL;
            }
        }else if (version == 3){
            ar(cereal::make_nvp("ModelType", gpType));
            if(gpType == GPType::GPLIGHT){
                GaussianProcessLight lgp;
                ar(cereal::make_nvp("GaussianProcessLight", lgp));
                this->mGP = std::make_shared<GaussianProcessLight>(lgp);
            }else if (gpType == GPType::GPNORMAL) {
                GaussianProcess gp;
                ar(cereal::make_nvp("GaussianProcess", gp));
                this->mGP = std::make_shared<GaussianProcess>(gp);
            }
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported version (version=" + std::to_string(version) +")"));
        }
        ar(CEREAL_NVP(mRssiStandardDeviations));
        mBeaconIdIndexMap = BLEBeacon::constructBeaconIdToIndexMap(mBLEBeacons);
        mStdevRssiForUnknownBeacon = computeNormalStandardDeviation(mRssiStandardDeviations);
        
        if(version <= 2){
            try{
                ar(cereal::make_nvp("mTDelay", mTDelay));
                this->tDelay(mTDelay);
            }catch(cereal::Exception& e){
                std::cerr << "mTDelay is not found (default value mTDelay=" << mTDelay << std::endl;
            }
        }else if(version == 3){
            ar(cereal::make_nvp("mTDelay", mTDelay));
            this->tDelay(mTDelay);
        }else{
            BOOST_THROW_EXCEPTION(LocException("unsupported version (version=" + std::to_string(version) +")"));
        }

    }
    
    //explicit instantiation
    template void GaussianProcessLDPLMultiModel<State, Beacons>::load<cereal::JSONInputArchive>(cereal::JSONInputArchive& archive);
    template void GaussianProcessLDPLMultiModel<State, Beacons>::save<cereal::JSONOutputArchive>(cereal::JSONOutputArchive& archive) const;
    template void GaussianProcessLDPLMultiModel<State, Beacons>::load<cereal::PortableBinaryInputArchive>(cereal::PortableBinaryInputArchive& archive);
    template void GaussianProcessLDPLMultiModel<State, Beacons>::save<cereal::PortableBinaryOutputArchive>(cereal::PortableBinaryOutputArchive& archive) const;

    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(std::ofstream& ofs, bool binary) {
        if (binary) {
            if (version < BINARY_SUPPORTED_MIN_VERSION) {
                version = BINARY_SUPPORTED_MIN_VERSION;
            }
            cereal::PortableBinaryOutputArchive oarchive(ofs);
            oarchive(*this);
        } else {
            cereal::JSONOutputArchive oarchive(ofs, cereal::JSONOutputArchive::Options::NoIndent());
            oarchive(*this);
        }
    }
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(std::ostringstream& oss, bool binary) {
        if (binary) {
            if (version < BINARY_SUPPORTED_MIN_VERSION) {
                version = BINARY_SUPPORTED_MIN_VERSION;
            }
            cereal::PortableBinaryOutputArchive oarchive(oss);
            oarchive(*this);
        } else {
            cereal::JSONOutputArchive oarchive(oss, cereal::JSONOutputArchive::Options::NoIndent());
            oarchive(*this);
        }
    }
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::save(cereal::PortableBinaryOutputArchive& oarchive, const std::string& name) {
        if (version < BINARY_SUPPORTED_MIN_VERSION) {
            version = BINARY_SUPPORTED_MIN_VERSION;
        }
        if(name.length()==0){
            oarchive(*this);
        }else{
            oarchive(cereal::make_nvp(name, *this));
        }
    }
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(std::ifstream& ifs, bool binary){
        if (binary) {
            cereal::PortableBinaryInputArchive iarchive(ifs);
            iarchive(*this);
        } else {
            cereal::JSONInputArchive iarchive(ifs);
            iarchive(*this);
        }
    }

    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(std::istringstream& iss, bool binary){
        if (binary) {
            cereal::PortableBinaryInputArchive iarchive(iss);
            iarchive(*this);
        } else {
            cereal::JSONInputArchive iarchive(iss);
            iarchive(*this);
        }
    }
    
    template<class Tstate, class Tinput>
    void GaussianProcessLDPLMultiModel<Tstate, Tinput>::load(cereal::PortableBinaryInputArchive& ar, const std::string& name){
        if(name.length()==0){
            ar(*this);
        }else{
            ar(cereal::make_nvp(name, *this));
        }
    }
    
    
    /**
     Implementation of GaussianProcessLDPLMultiModelTrainer
     **/
    template<class Tstate, class Tinput>
    GaussianProcessLDPLMultiModel<Tstate, Tinput>* GaussianProcessLDPLMultiModelTrainer<Tstate, Tinput>::train(){
        
        Samples samples = mDataStore->getSamples();
        BLEBeacons bleBeacons = mDataStore->getBLEBeacons();
        if(bleBeacons.size()<=0){
            BOOST_THROW_EXCEPTION(LocException("BLEBeacons have not been set to dataStore."));
        }
        Samples samplesFiltered;
        try{
            samplesFiltered = Sample::filterUnregisteredBeacons(samples, bleBeacons);
        }catch(LocException& ex) {
            throw ex;
        }
        GaussianProcessLDPLMultiModel<Tstate, Tinput>* obsModel = new GaussianProcessLDPLMultiModel<Tstate, Tinput>();
        
        obsModel->gpType = gpType;
        obsModel->matType = matType;
        
        obsModel->bleBeacons(bleBeacons);
        obsModel->train(samplesFiltered);
        
        return obsModel;
    }
    
    
    //Explicit instantiation
    template class GaussianProcessLDPLMultiModel<State, Beacons>;
    template class GaussianProcessLDPLMultiModelTrainer<State, Beacons>;
    
}
