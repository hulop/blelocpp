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

#include "KernelFunction.hpp"
#include "SerializeUtils.hpp"

GaussianKernel::GaussianKernel(Parameters params){
    this->params = params;
    this->variance_ = params.sigma_f*params.sigma_f;
}

double GaussianKernel::computeKernel(const double x1[], const double x2[]) const{
    double sqsum = 0;
    for(int i=0; i<ndim; i++){
        double diff = (x1[i] - x2[i])/params.lengthes[i];
        sqsum += diff*diff;
    }
    double kernel = variance_ * std::exp(-sqsum);
    return kernel;
}

double GaussianKernel::variance() const{
    return variance_;
}

double GaussianKernel::sqsum(const double x1[], const double x2[]) const {
    double sqsum = 0;
    for(int i=0; i<ndim; i++){
        double diff = (x1[i] - x2[i])/params.lengthes[i];
        sqsum += diff*diff;
    }
    return sqsum;
}

template<class Archive>
void GaussianKernel::Parameters::serialize(Archive& ar){
    ar(CEREAL_NVP(sigma_f));
    ar(CEREAL_NVP(lengthes));
}

template void GaussianKernel::Parameters::serialize<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive);
template void GaussianKernel::Parameters::serialize<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive);
template void GaussianKernel::Parameters::serialize<cereal::PortableBinaryInputArchive> (cereal::PortableBinaryInputArchive& archive);
template void GaussianKernel::Parameters::serialize<cereal::PortableBinaryOutputArchive> (cereal::PortableBinaryOutputArchive& archive);

template<class Archive>
void GaussianKernel::save(Archive& ar) const{
    ar(CEREAL_NVP(params));
}

template<class Archive>
void GaussianKernel::load(Archive& ar){
    ar(CEREAL_NVP(params));
    variance_ = params.sigma_f * params.sigma_f;
}

template void GaussianKernel::save<cereal::JSONOutputArchive> (cereal::JSONOutputArchive& archive) const;
template void GaussianKernel::load<cereal::JSONInputArchive> (cereal::JSONInputArchive& archive);
template void GaussianKernel::save<cereal::PortableBinaryOutputArchive> (cereal::PortableBinaryOutputArchive& archive) const;
template void GaussianKernel::load<cereal::PortableBinaryInputArchive> (cereal::PortableBinaryInputArchive& archive);

std::string GaussianKernel::Parameters::toString() const{
    std::stringstream ss;
    ss << sigma_f <<"," << lengthes[0] << "," << lengthes[1] <<","<< lengthes[2] << "," <<lengthes[3];
    return ss.str();
}


CEREAL_REGISTER_TYPE(GaussianKernel);
