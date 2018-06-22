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

#include "CnnPoseUtil.hpp"
#include "Pose.hpp"

namespace loc {
    
    double CnnPoseUtil::convertQuaternion2Yaw(Eigen::Quaterniond& quaternion){
        // NavCog yaw orientation origin is same direction to X-axis in X-Y plane,
        // and represented in spherical coordinate (left hand coordinate, plus and minus is opposite)
        // CNN yaw orientation origin is same direction to Y-axis in X-Y plane, and right hand coordinate.
        // Subtract 90 degrees then multiply -1 to align CNN coordinate to NavCog coordinate
        Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
        return Pose::normalizeOrientaion(-1.0 * (euler[2] - M_PI*(90.0/180.0)));
    }
    
}
