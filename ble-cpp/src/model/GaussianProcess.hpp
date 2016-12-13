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

#ifndef GaussianProcess_hpp
#define GaussianProcess_hpp

#include <stdio.h>
#include <iostream>
#include <vector>
#include <memory>
#include <complex>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include "KernelFunction.hpp"
#include "MathUtils.hpp"

namespace loc{
    
    class GaussianProcessParameterSet{
    public:
        std::vector<double> sigmaFs{1,2,3,5};
        std::vector<double> lengthes{1,2,3,4,5,7,9};
        std::vector<double> lengthFloors{0.01};
        std::vector<double> sigmaNs{1};
    };
    
    class GaussianProcessParameters{
    public:
        GaussianKernel::Parameters gaussianKernelParameters;
        double sigmaN;
    };
    
    class GaussianProcess{
        
    private:
        // variables to be serialized
        ////std::shared_ptr<KernelFunction> mKernel;
        GaussianKernel mGaussianKernel;
        Eigen::MatrixXd X_;
        Eigen::MatrixXd Weights_;
        double sigmaN_ = 1.0;
        
        // variables not to be serialized
        Eigen::MatrixXd Y_;
        Eigen::MatrixXd K_;
        Eigen::MatrixXd Ky_;
        Eigen::MatrixXd invKy_;
        Eigen::MatrixXd Actives_;
        GaussianProcessParameterSet mParameterSet;
        
    public:
        // A function for serealization
        template<class Archive>
        void serialize(Archive& ar);
        
        GaussianProcess& sigmaN(double sigmaN);
        double sigmaN() const;
        /*
        GaussianProcess& kernel(std::shared_ptr<KernelFunction> kernel){
            mKernel = kernel;
            return *this;
        }
        */
        GaussianProcess& gaussianProcessParameterSet(const GaussianProcessParameterSet&);
        GaussianProcess& gaussianKernel(GaussianKernel gaussianKernel);
        GaussianKernel gaussianKernel() const;
        
        Eigen::MatrixXd X() const;
        Eigen::MatrixXd Y() const;
        GaussianProcess& fit(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y);
        GaussianProcess& fit(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives);
        GaussianProcess& actives(const Eigen::MatrixXd& Actives);
        
        Eigen::MatrixXd computeKernelMatrix(const Eigen::MatrixXd& X);
        Eigen::VectorXd computeKstar(double x[]) const;
        
        Eigen::VectorXd predict(double x[]) const;
        Eigen::VectorXd predict(const Eigen::VectorXd& kstar) const;
        
        double predict(double x[], int index);
        std::vector<double> predict(double x[], const std::vector<int>& indices) const;
        std::vector<double> predict(const Eigen::VectorXd& kstar, const std::vector<int>& indices) const;
        Eigen::VectorXd predictVarianceF(double x[]) const;
        Eigen::VectorXd predictVarianceF(const Eigen::VectorXd& kstar) const;
        
        double computeLogLikelihood(double x[], const Eigen::VectorXd& y) const;
        double marginalLogLikelihood();
        double predictiveLogLikelihood();
        double leaveOneOutMSE();
        
        std::vector<GaussianProcessParameters> createParameterMatrix(const GaussianProcessParameterSet&) const;
        void fitCV(const Eigen::MatrixXd & X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives);
    };
}

#endif /* GaussianProcess_hpp */
