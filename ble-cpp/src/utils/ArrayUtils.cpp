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

#include "ArrayUtils.hpp"

void ArrayUtils::normalize(double outArray[], const double inArray[], int n){
    double sum = 0;
    for(int i=0; i<n; i++){
        sum+=inArray[i];
    }
    for(int i=0; i<n; i++){
        outArray[i] = inArray[i]/sum;
    }
}

std::vector<double> ArrayUtils::arrayToVector(double *array){
    size_t size = sizeof(array)/sizeof(array[0]);
    return std::vector<double>(array, array + size);
}

std::vector<double> ArrayUtils::computeWeightsFromLogLikelihood(std::vector<double> logLikelihoods){
    size_t n = logLikelihoods.size();
    std::vector<double> weights(n);
    double maxLogLL = *std::max_element(logLikelihoods.begin(), logLikelihoods.end());
    double sum = 0;
    for(int i=0; i<n; i++){
        double tmp = exp(logLikelihoods.at(i)-maxLogLL);
        weights[i] = tmp;
        sum+=tmp;
    }
    for(int i=0; i<n; i++){
        weights[i] = weights[i]/(sum);
    }
    return weights;
}

Eigen::VectorXd ArrayUtils::vectorToEigenVector(std::vector<double> v){
    assert(v.size() <= std::numeric_limits<int>::max());
    const int n = static_cast<int>(v.size());
    Eigen::VectorXd V = Eigen::Map<Eigen::VectorXd>(&v[0], n, 1 );
    return V;
}
std::vector<double> ArrayUtils::eigenVectorToEigen(Eigen::VectorXd V){
    assert(V.size() <= std::numeric_limits<int>::max());
    const int n = static_cast<int>(V.size());
    std::vector<double> v(n);
    Eigen::Map<Eigen::VectorXd>(&v[0], n, 1 ) = V;
    return v;
}