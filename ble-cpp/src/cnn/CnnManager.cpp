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

#include "CnnManager.hpp"
#include "TensorFlowUtil.hpp"

namespace loc{
    
    CnnManager::CnnManager() {
        mIsInitialized = false;
    }
    
    void CnnManager::init(const std::string& modelPath, loc::ImageLocalizeMode imageLocalizeMode) {
        close();
        
        std::lock_guard<std::mutex> lock(mutex);
        
        tensorflow::Status load_status = TensorFlowUtil::loadModelFromPath(modelPath, &tfSession);
        if (!load_status.ok()) {
            std::cerr << "Couldn't load model: " << load_status;
        }
        mImageLocalizeMode = imageLocalizeMode;
        mIsInitialized = true;
    }

    void CnnManager::close() {
        std::lock_guard<std::mutex> lock(mutex);

        if (mIsInitialized) {
            std::cerr << "Close old tensorflow session " << std::endl;
            tensorflow::Status close_status = tfSession->Close();
            if (!close_status.ok()) {
                std::cerr << "Couldn't close tensorflow session " << close_status;
            }
        }
        mIsInitialized = false;
    }
    
    bool CnnManager::isInitialized() {
        return mIsInitialized;
    }
    
    std::vector<double> CnnManager::runImageCnn(const tensorflow::Tensor& imageTensor,
                                                bool useMobileNet, bool useLstm) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (!mIsInitialized) {
            return std::vector<double>();
        }
        
        std::string input_image_layer_name;
        std::string output_pos_layer_name;
        std::string output_rot_layer_name;
        if (!useMobileNet && !useLstm) {
            input_image_layer_name = "input_1";
            output_pos_layer_name = "cls3_fc_pose_xyz/BiasAdd";
            output_rot_layer_name = "cls3_fc_pose_wpqr/BiasAdd";
        } else if (useMobileNet && !useLstm) {
            input_image_layer_name = "input_1";
            output_pos_layer_name = "cls_fc_pose_xyz/BiasAdd";
            output_rot_layer_name = "cls_fc_pose_wpqr/BiasAdd";
        } else {
            input_image_layer_name = "input_1";
            output_pos_layer_name = "lstm_pose_xyz/Reshape_1";
            output_rot_layer_name = "lstm_pose_wpqr/Reshape_1";
        }
        
        std::vector<double> result;
        
        if (tfSession.get()) {
            std::vector<tensorflow::Tensor> outputs;
             tensorflow::Status run_status = tfSession->Run({{input_image_layer_name, imageTensor}}, {output_pos_layer_name, output_rot_layer_name}, {}, &outputs);
            if (!run_status.ok()) {
                std::cerr << "Running model failed:" << run_status;
            } else {
                tensorflow::Tensor *output_pos = &outputs[0];
                tensorflow::Tensor *output_rot = &outputs[1];
                auto predictions_pos = output_pos->flat<float>();
                auto predictions_rot = output_rot->flat<float>();
                
                for (int index = 0; index < predictions_pos.size(); index += 1) {
                    const float predictionValue = predictions_pos(index);
                    result.push_back(predictionValue);
                }
                for (int index = 0; index < predictions_rot.size(); index += 1) {
                    const float predictionValue = predictions_rot(index);
                    result.push_back(predictionValue);
                }
            }
        }
        
        return result;
    }
    
    std::vector<double> CnnManager::runBeaconCnn(const tensorflow::Tensor& beaconTensor,
                                                 bool useLstm) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (!mIsInitialized) {
            return std::vector<double>();
        }
        
        std::string input_beacon_layer_name;
        std::string output_pos_layer_name;
        std::string output_rot_layer_name;
        if (!useLstm) {
            input_beacon_layer_name = "input_1";
            output_pos_layer_name = "beacon_cls3_fc_pose_xyz/BiasAdd";
            output_rot_layer_name = "beacon_cls3_fc_pose_wpqr/BiasAdd";
        } else {
            input_beacon_layer_name = "input_1";
            output_pos_layer_name = "beacon_lstm_pose_xyz/Reshape_1";
            output_rot_layer_name = "beacon_lstm_pose_wpqr/Reshape_1";
        }
        
        std::vector<double> result;
        
        if (tfSession.get()) {
            std::vector<tensorflow::Tensor> outputs;
            tensorflow::Status run_status = tfSession->Run({{input_beacon_layer_name, beaconTensor}}, {output_pos_layer_name, output_rot_layer_name}, {}, &outputs);
            if (!run_status.ok()) {
                std::cerr << "Running model failed:" << run_status;
            } else {
                tensorflow::Tensor *output_pos = &outputs[0];
                tensorflow::Tensor *output_rot = &outputs[1];
                auto predictions_pos = output_pos->flat<float>();
                auto predictions_rot = output_rot->flat<float>();
                
                for (int index = 0; index < predictions_pos.size(); index += 1) {
                    const float predictionValue = predictions_pos(index);
                    result.push_back(predictionValue);
                }
                for (int index = 0; index < predictions_rot.size(); index += 1) {
                    const float predictionValue = predictions_rot(index);
                    result.push_back(predictionValue);
                }
            }
        }
        
        return result;
    }
    
    std::vector<double> CnnManager::runImageBeaconCnn(const tensorflow::Tensor& imageTensor,
                                                      const tensorflow::Tensor& beaconTensor,
                                                      bool useMobileNet, bool useLstm) {
        std::lock_guard<std::mutex> lock(mutex);
        
        if (!mIsInitialized) {
            return std::vector<double>();
        }
        
        std::string input_image_layer_name;
        std::string input_beacon_layer_name;
        std::string output_pos_layer_name;
        std::string output_rot_layer_name;
        if (!useMobileNet && !useLstm) {
            input_image_layer_name = "input_1";
            input_beacon_layer_name = "input_2";
            output_pos_layer_name = "image_beacon_cls3_fc_pose_xyz/BiasAdd";
            output_rot_layer_name = "image_beacon_cls3_fc_pose_wpqr/BiasAdd";
        } else if (useMobileNet && !useLstm) {
            input_image_layer_name = "input_1";
            input_beacon_layer_name = "input_2";
            output_pos_layer_name = "image_beacon_cls_fc_pose_xyz/BiasAdd";
            output_rot_layer_name = "image_beacon_cls_fc_pose_wpqr/BiasAdd";
        } else {
            input_image_layer_name = "input_1";
            input_beacon_layer_name = "input_2";
            output_pos_layer_name = "image_beacon_lstm_pose_xyz/Reshape_1";
            output_rot_layer_name = "image_beacon_lstm_pose_wpqr/Reshape_1";
        }
        
        std::vector<double> result;

        if (tfSession.get()) {
            std::vector<tensorflow::Tensor> outputs;
            tensorflow::Status run_status = tfSession->Run({{input_image_layer_name, imageTensor}, {input_beacon_layer_name, beaconTensor}}, {output_pos_layer_name, output_rot_layer_name}, {}, &outputs);
            if (!run_status.ok()) {
                std::cerr << "Running model failed:" << run_status;
            } else {
                tensorflow::Tensor *output_pos = &outputs[0];
                tensorflow::Tensor *output_rot = &outputs[1];
                auto predictions_pos = output_pos->flat<float>();
                auto predictions_rot = output_rot->flat<float>();
                
                for (int index = 0; index < predictions_pos.size(); index += 1) {
                    const float predictionValue = predictions_pos(index);
                    result.push_back(predictionValue);
                }
                for (int index = 0; index < predictions_rot.size(); index += 1) {
                    const float predictionValue = predictions_rot(index);
                    result.push_back(predictionValue);
                }
            }
        }
        
        return result;
    }

    std::vector<double> CnnManager::runCnn(const tensorflow::Tensor& imageTensor,
                                           const tensorflow::Tensor& beaconTensor,
                                           bool useMobileNet, bool useLstm) {
        std::vector<double> result;
        if (mImageLocalizeMode==ImageLocalizeMode::IMAGE) {
            result = runImageCnn(imageTensor, useMobileNet, useLstm);
        } else if (mImageLocalizeMode==ImageLocalizeMode::BEACON) {
            result = runBeaconCnn(beaconTensor, useLstm);
        } else if (mImageLocalizeMode==ImageLocalizeMode::IMAGE_BEACON) {
            result = runImageBeaconCnn(imageTensor, beaconTensor, useMobileNet, useLstm);
        } else {
            assert(false);  // Unknown image localize mode
        }
        return result;
    }
}

