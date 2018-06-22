/*******************************************************************************
 * Copyright (c) 2018  IBM Corporation, Carnegie Mellon University and others
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

#include "ImageLocalizeObservationModel.hpp"
#include <Eigen/Geometry>

namespace loc{
    
    template<class Tstate, class Tinput>
    double ImageLocalizeObservationModel<Tstate, Tinput>::computeLogLikelihood(const Tstate& state, const Tinput& input){
        std::vector<double> values = this->computeLogLikelihoodRelatedValues(state, input);
        return values.at(0);
    }
    
    template<class Tstate, class Tinput>
    std::vector<double> ImageLocalizeObservationModel<Tstate, Tinput>::computeLogLikelihoodRelatedValues(const Tstate& state, const Tinput& input){
        //Assuming Tinput = Pose
        
        std::vector<double> returnValues(4); // logLikelihood, mahalanobisDistance, #knownBeacons, #unknownBeacons
        
        const Pose* pInput = dynamic_cast<const Pose*>(&input);
        const State* pState = dynamic_cast<const State*>(&state);
        
        Eigen::VectorXd inputvec(3);
        inputvec << pInput->x(), pInput->y(), pInput->z();
        assert(!inputvec.hasNaN());
        Eigen::VectorXd statevec(3);
        statevec << pState->x(), pState->y(), pState->z();
        assert(!statevec.hasNaN());
        Eigen::VectorXd diffvec = inputvec - statevec;
        assert(!diffvec.hasNaN());
        double dist = diffvec.norm();
        assert(!isnan(dist));
        
        double logLL = normFunc(dist, 0.0, mSigmaDistImageLikelihood);
        assert(!isnan(logLL));
        double mahaDist = MathUtils::mahalanobisDistance(dist, 0.0, mSigmaDistImageLikelihood);
        assert(!isnan(mahaDist));
        
        Eigen::Quaternionf inputQuat = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(pInput->orientation(), Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf stateQuat = Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(0.0, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(pState->orientation(), Eigen::Vector3f::UnitZ());
        float orientationDist = inputQuat.angularDistance(stateQuat);
        
        double orientationLogLL = normFunc(orientationDist, 0.0, M_PI*mSigmaAngleImageLikelihood/180.0);
        assert(!isnan(orientationLogLL));
        double orientationMahaDist = MathUtils::mahalanobisDistance(orientationDist, 0.0, M_PI*mSigmaAngleImageLikelihood/180.0);
        assert(!isnan(mahaDist));
        
        returnValues[0] = logLL + orientationLogLL;
        returnValues[1] = sqrt(pow(mahaDist,2)+pow(orientationMahaDist,2));
        returnValues[2] = 1; // corresponds to count known beacon for beacon observation
        returnValues[3] = 0; // corresponds to count unknown beacon for beacon observation
        
        return returnValues;
    }
    
    template<class Tstate, class Tinput>
    std::vector<double> ImageLocalizeObservationModel<Tstate, Tinput>::computeLogLikelihood(const std::vector<Tstate> & states, const Tinput & input) {
        int n = (int) states.size();
        std::vector<double> logLLs(n);
        for(int i=0; i<n; i++){
            logLLs[i] = this->computeLogLikelihood(states.at(i), input);
            assert(!isnan(logLLs[i]));
        }
        return logLLs;
    }
    
    template<class Tstate, class Tinput>
    std::vector<std::vector<double>> ImageLocalizeObservationModel<Tstate, Tinput>::computeLogLikelihoodRelatedValues(const std::vector<Tstate> & states, const Tinput & input) {
        int n = (int) states.size();
        std::vector<double> logLLs(n);
        
        std::vector<std::vector<double>> values(n);
        for(int i=0; i<n; i++){
            values[i] = this->computeLogLikelihoodRelatedValues(states.at(i), input);
        }
        return values;
    }
    
    template<class Tstate, class Tinput>
    ImageLocalizeObservationModel<Tstate, Tinput>& ImageLocalizeObservationModel<Tstate, Tinput>::sigmaDistImageLikelihood(double dist) {
        mSigmaDistImageLikelihood = dist;
        return *this;
    }

    template<class Tstate, class Tinput>
    double ImageLocalizeObservationModel<Tstate, Tinput>::sigmaDistImageLikelihood() const {
        return mSigmaDistImageLikelihood;
    }

    template<class Tstate, class Tinput>
    ImageLocalizeObservationModel<Tstate, Tinput>& ImageLocalizeObservationModel<Tstate, Tinput>::sigmaAngleImageLikelihood(double angle) {
        mSigmaAngleImageLikelihood = angle;
        return *this;
    }
    
    template<class Tstate, class Tinput>
    double ImageLocalizeObservationModel<Tstate, Tinput>::sigmaAngleImageLikelihood() const {
        return mSigmaAngleImageLikelihood;
    }
    
    //Explicit instantiation
    template class ImageLocalizeObservationModel<State, Pose>;
    
}
