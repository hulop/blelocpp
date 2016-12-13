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

#include "GaussianProcess.hpp"
#include "ArrayUtils.hpp"
#include "SerializeUtils.hpp"

namespace loc{

    template<class Archive>
    void GaussianProcess::serialize(Archive& ar){
        ar(CEREAL_NVP(sigmaN_));
        // ar(CEREAL_NVP(mKernel));
        ar(CEREAL_NVP(mGaussianKernel));
        ar(CEREAL_NVP(X_));
        ar(CEREAL_NVP(Weights_));
    }
    // Explicit instanciation
    template void GaussianProcess::serialize<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive);
    template void GaussianProcess::serialize<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive);
    
    GaussianProcess& GaussianProcess::sigmaN(double sigmaN){
        sigmaN_ = sigmaN;
        return *this;
    }
    
    double GaussianProcess::sigmaN() const{
        return sigmaN_;
    }
    
    /*
     GaussianProcess& GaussianProcess::kernel(std::shared_ptr<KernelFunction> kernel){
     mKernel = kernel;
     return *this;
     }
     */
    
    GaussianProcess& GaussianProcess::gaussianKernel(GaussianKernel gaussianKernel){
        mGaussianKernel = gaussianKernel;
        return *this;
    }
    
    GaussianKernel GaussianProcess::gaussianKernel() const{
        return mGaussianKernel;
    }
    
    GaussianProcess& GaussianProcess::gaussianProcessParameterSet(const GaussianProcessParameterSet& gpParamsSet){
        mParameterSet = gpParamsSet;
        return *this;
    }
    
    Eigen::MatrixXd GaussianProcess::X() const{
        return X_;
    }
    
    Eigen::MatrixXd GaussianProcess::Y() const{
        return Y_;
    }
    
    GaussianProcess& GaussianProcess::fit(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y){
        size_t n = Y_.rows();
        size_t ny = Y_.cols();
        Eigen::MatrixXd Actives = Eigen::MatrixXd::Constant(n, ny, 1.0);
        return fit(X,Y,Actives);
    }
    
    GaussianProcess& GaussianProcess::fit(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives){
        actives(Actives);
        X_ = X;
        Y_ = Y;
        size_t ns = X.rows();
        
        K_ = computeKernelMatrix(X);
        Eigen::MatrixXd sigmaNmat = (sigmaN_*sigmaN_) * Eigen::MatrixXd::Identity(ns,ns);
        
        Ky_ = K_ + sigmaNmat;
        invKy_ = Ky_.inverse();
        
        Weights_ = invKy_*Y_;
        
        return *this;
    }
    
    GaussianProcess& GaussianProcess::actives(const Eigen::MatrixXd &Actives){
        Actives_ = Actives;
        return *this;
    }
    
    Eigen::MatrixXd GaussianProcess::computeKernelMatrix(const Eigen::MatrixXd& X){
        long n = X.rows();
        size_t nx = X.cols();
        Eigen::MatrixXd K(n,n);
        
        double* x1 = new double[nx];
        double* x2 = new double[nx];
        
        for(int i=0; i<n; i++){
            for(int k=0; k<nx; k++){
                x1[k] = X(i,k);
            }
            for(int j=i; j<n; j++){
                for(int k=0; k<nx; k++){
                    x2[k] = X(j,k);
                }
                // double k = mKernel->computeKernel(x1, x2);
                double k = mGaussianKernel.computeKernel(x1, x2);
                K(i,j)=k;
                K(j,i)=k;
            }
        }
        
        delete[] x1;
        delete[] x2;
        
        return K;
    }
    
    Eigen::VectorXd GaussianProcess::computeKstar(double x[]) const{
        size_t n = X_.rows();
        size_t nx = X_.cols();
        
        Eigen::VectorXd kstar = Eigen::VectorXd(n);
        
        double* x_i;
        x_i = new double[nx];
        for(int i=0; i<n; i++){
            for(int j=0; j<nx; j++){
                x_i[j]=X_(i,j);
            }
            //double k = mKernel->computeKernel(x, x_i);
            double k = mGaussianKernel.computeKernel(x, x_i);
            kstar(i) = k;
        }
        delete x_i;
        
        return kstar;
    }
    
    Eigen::VectorXd GaussianProcess::predict(double x[]) const{
        Eigen::VectorXd kstar = computeKstar(x);
        return predict(kstar);
    }
    
    Eigen::VectorXd GaussianProcess::predict(const Eigen::VectorXd& kstar) const{
        Eigen::VectorXd ypred = Weights_.transpose()*(kstar);
        return ypred;
    }
    
    double GaussianProcess::predict(double x[], int index){
        std::vector<int> indices(1);
        indices[0] = index;
        return predict(x,indices)[0];
    }
    
    std::vector<double> GaussianProcess::predict(double x[], const std::vector<int>& indices) const{
        Eigen::VectorXd kstar = computeKstar(x);
        return predict(kstar, indices);
    }
    
    std::vector<double> GaussianProcess::predict(const Eigen::VectorXd& kstar, const std::vector<int>& indices) const{
        size_t m = indices.size();
        std::vector<double> ypreds(m);
        for(int i=0; i<m; i++){
            int index = indices.at(i);
            Eigen::VectorXd ypred = (Weights_.col(index).transpose())*(kstar);
            ypreds[i]=ypred(0);
        }
        return ypreds;
    }
    
    Eigen::VectorXd GaussianProcess::predictVarianceF(double x[]) const{
        Eigen::VectorXd kstar = computeKstar(x);
        return predictVarianceF(kstar);
    }
    
    Eigen::VectorXd GaussianProcess::predictVarianceF(const Eigen::VectorXd& kstar) const{
        //Eigen::VectorXd varianceF = mKernel->variance() - ((kstar.transpose())*invKy_*(kstar)).array();
        Eigen::VectorXd varianceF = mGaussianKernel.variance() - ((kstar.transpose())*invKy_*(kstar)).array();
        return varianceF;
    }
    
    double GaussianProcess::computeLogLikelihood(double x[], const Eigen::VectorXd& y) const{
        Eigen::VectorXd kstar = computeKstar(x);
        
        Eigen::VectorXd ypred = predict(kstar);
        Eigen::VectorXd varianceFs = predictVarianceF(kstar);
        
        size_t m = ypred.size();
        
        double logLL=0;
        for(int i=0; i<m; i++){
            double value = y(i);
            double mean = ypred(i);
            double stdev = sqrt(varianceFs(i) + sigmaN_*sigmaN_);
            logLL += MathUtils::logProbaNormal(value, mean, stdev);
        }
        return logLL;
    }
    
    double GaussianProcess::marginalLogLikelihood(){
    
        size_t n = Y_.rows();
        size_t m = Y_.cols();
        double sumMarginalLogLL = 0;
        
        double logdetKy = 0;
        Eigen::VectorXcd eigs = Ky_.eigenvalues();
        
        for(int i=0; i<eigs.size(); i++){
            std::complex<double> eig = eigs(i);
            logdetKy += log(eig.real());
        }
        
        // compute marginal log-likelihood for each BLE beacon
        for(int i=0; i<m; i++){
            Eigen::VectorXd y = Y_.col(i);
            double marginalLogLL = - 0.5*y.transpose()*invKy_*y - 0.5*logdetKy - 0.5*n*log(2*M_PI);
            sumMarginalLogLL += marginalLogLL;
        }
        return sumMarginalLogLL;
    }
    
    double GaussianProcess::predictiveLogLikelihood(){
        size_t n = Y_.rows();
        size_t m = Y_.cols();
        
        double sumPredLogLL = 0;
        for(int j=0; j<m; j++){
            double predLogLL_j = 0;
            Eigen::VectorXd invKy_y = invKy_*(Y_.col(j));
            for(int i=0; i<n; i++){
                double y = Y_(i,j);
                if(Actives_(i,j)==1){
                    double mu = y - invKy_y(i)/invKy_(i,i);
                    double sigma_p2 = 1.0/invKy_(i,i);
                    double sigma_p = sqrt(sigma_p2);
                    double predLogLL_j_i = MathUtils::logProbaNormal(y, mu, sigma_p);
                    predLogLL_j += predLogLL_j_i;
                }
            }
            sumPredLogLL += predLogLL_j;
        }
        return sumPredLogLL;
    }
    
    /**
     Compute leave-one-out MSE. (Note) LOO-MSE does not depend on the scale.
     **/
    double GaussianProcess::leaveOneOutMSE(){
        
        size_t n = Y_.rows();
        size_t m = Y_.cols();
        
        Eigen::MatrixXd Hmat =K_*invKy_;
        Eigen::MatrixXd Ypred = Hmat*Y_;
        
        double sumSquareError = 0;
        int count = 0;
        for(int j=0; j<m; j++){
            for(int i=0; i<n; i++){
                if(Actives_(i,j)==1){
                    double y = Y_(i,j);
                    double ypred = Ypred(i,j);
                    double ei = y - ypred;
                    double diff = ei/(1.0-Hmat(i,i));
                    double errorcv = diff*diff;
                    sumSquareError += errorcv;
                    count++;
                }
            }
        }
        sumSquareError/=count;
        return sumSquareError;
    }

    std::vector<GaussianProcessParameters> GaussianProcess::createParameterMatrix(const GaussianProcessParameterSet& paramsSet) const{
        std::vector<GaussianProcessParameters> paramsMat;
        for(double sigmaF: paramsSet.sigmaFs){
            for(double length: paramsSet.lengthes){
                for(double lengthFloor: paramsSet.lengthFloors){
                    for(double sigmaN: paramsSet.sigmaNs){
                        GaussianKernel::Parameters gkParams;
                        gkParams.sigma_f = sigmaF;
                        gkParams.lengthes[0] = length;
                        gkParams.lengthes[1] = length;
                        gkParams.lengthes[2] = length;
                        gkParams.lengthes[3] = lengthFloor;
                        GaussianProcessParameters gpParams;
                        gpParams.gaussianKernelParameters = gkParams;
                        gpParams.sigmaN = sigmaN;
                        paramsMat.push_back(gpParams);
                    }
                }
            }
        }
        return paramsMat;
    }
    
    void GaussianProcess::fitCV(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives){
        std::vector<GaussianProcessParameters> gkParamsMatrix
                = createParameterMatrix(mParameterSet);
        size_t nEval = gkParamsMatrix.size();
        double minValue = std::numeric_limits<double>::max();
        std::vector<double> gkParamsRowMin(6);
        
        int indexMinError = 0;
        for(int i=0; i<nEval; i++){
            GaussianKernel::Parameters gkParams = gkParamsMatrix.at(i).gaussianKernelParameters;
            double sigma_n = gkParamsMatrix.at(i).sigmaN;
            this->sigmaN(sigma_n);
            GaussianKernel gKernel(gkParams);
            this->gaussianKernel(gKernel);
            // fit GP
            this->fit(X,Y,Actives);
            
            double looMSE = this->leaveOneOutMSE();
            std::cout << "LOOMSE=" << looMSE;
            std::cout << ", (kernel parameters=" << gkParams.toString() << "," << sigma_n << std::endl;
            if(looMSE < minValue){
                minValue = looMSE;
                indexMinError = i;
                std::cout << "Min LOOMSE updated." << std::endl;
            }
        }
        
        // Fit this model with the selected parameters.
        GaussianKernel::Parameters gkParamsMin = gkParamsMatrix.at(indexMinError).gaussianKernelParameters;
        double sigma_n_min = gkParamsMatrix.at(indexMinError).sigmaN;
        this->sigmaN(sigma_n_min);
        // std::shared_ptr<KernelFunction> kernel(new GaussianKernel(gkParamsMin));
        // mKernel = kernel;
        GaussianKernel gKernel(gkParamsMin);
        this->gaussianKernel(gKernel);
        
        this->fit(X,Y,Actives);
    }
}
