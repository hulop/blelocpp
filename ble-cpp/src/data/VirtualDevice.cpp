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

#include "VirtualDevice.hpp"

namespace loc{
    
    double VirtualDevice::currentError() const{
        return mCurrentError;
    }
    
    void VirtualDevice::setStdevPose(Pose stdevPose){
        stdevPose_ = stdevPose;
        std::cout << "standard deviation of Pose was set. " << stdevPose_  << std::endl;
    }
    
    void VirtualDevice::csvPath(std::string csvPath){
        mCsvPath = csvPath;
    }
    
    void VirtualDevice::resultDir(std::string resultDir){
        mResultDir = resultDir;
    }
    
    void VirtualDevice::streamLocalizer(std::shared_ptr<StreamLocalizer> streamLocalizer){
        this->mStreamLocalizer = streamLocalizer;
    }
    
    std::shared_ptr<StreamLocalizer> VirtualDevice::streamLocalizer() const{
        return this->mStreamLocalizer;
    }
    
    void VirtualDevice::resetLocalizer(){
        mStreamLocalizer->resetStatus();
    }
    
    void VirtualDevice::checkTimestampConsistency(std::string str){
        long timestamp;
        sscanf(str.c_str(), "%ld,", &timestamp);
        checkTimestampConsistency(timestamp);
    }
    
    void VirtualDevice::checkTimestampConsistency(long timestamp){
        static long prevTimestamp = 0;
        if(prevTimestamp == 0){
            prevTimestamp = timestamp;
        }
        if(timestamp + timestampMargin < prevTimestamp){ // If new timestamp is older than previous timestamp
            std::cout << "Inconsistent timestamp(previousTimestamp=" << prevTimestamp << ", timestamp=" << timestamp << ") was inputted. Localizer status was reset." << std::endl;
            resetLocalizer();
        }
        prevTimestamp = timestamp;
    }
    
    // Interface method called by JNI
    Status VirtualDevice::update(Status status, std::string strBuffer){
        
        if(DataUtils::csvCheckAcceleration(strBuffer)){
            Acceleration acc = DataUtils::parseAccelerationCSV(strBuffer);
            mStreamLocalizer->putAcceleration(acc);
        }else if(DataUtils::csvCheckAttitude(strBuffer)){
            Attitude att = DataUtils::parseAttitudeCSV(strBuffer);
            mStreamLocalizer->putAttitude(att);
        }else if(DataUtils::csvCheckBeacons(strBuffer)){
            checkTimestampConsistency(strBuffer);
            Beacons beacons = DataUtils::parseBeaconsCSV(strBuffer);
            mStreamLocalizer->putBeacons(beacons);
        }else if(DataUtils::csvCheckSensorType(strBuffer, "Reset")){
            Pose poseReset = DataUtils::parseResetPoseCSV(strBuffer);
            //mStreamLocalizer->resetStatus(poseReset);
            mStreamLocalizer->resetStatus(poseReset, stdevPose_);
        }
        Status* statusUpdated = mStreamLocalizer->getStatus();
        
        return Status(*statusUpdated);
    }
    
    void VirtualDevice::processLine(std::string strBuffer){
        static bool wasReset = false;
        if(DataUtils::csvCheckAcceleration(strBuffer)){
            Acceleration acc = DataUtils::parseAccelerationCSV(strBuffer);
            mStreamLocalizer->putAcceleration(acc);
            Status* status = mStreamLocalizer->getStatus();
            static std::shared_ptr<Pose> posePred;
            posePred = status->meanPose();
            static std::shared_ptr<Pose> previousPosePred;
            if(!previousPosePred){
                previousPosePred = posePred;
            }
            if( ! Location::equals(*posePred, *previousPosePred)){
                long timestamp = acc.timestamp();
                std::cout << "Pre. ts="<<timestamp<< ", posePre=" << *posePred << std::endl;
                previousPosePred = posePred;
            }
            
        }else if(DataUtils::csvCheckAttitude(strBuffer)){
            Attitude att = DataUtils::parseAttitudeCSV(strBuffer);
            mStreamLocalizer->putAttitude(att);
            
        }else if(DataUtils::csvCheckBeacons(strBuffer)){
            checkTimestampConsistency(strBuffer);
            
            Beacons beacons = DataUtils::parseBeaconsCSV(strBuffer);
            mStreamLocalizer->putBeacons(beacons);
            
            Status* status = mStreamLocalizer->getStatus();
            //std::shared_ptr<Location> locEst = status->meanLocation();
            std::shared_ptr<Pose> poseEst = status->meanPose();
            try{
                Sample smp = DataUtils::parseSampleCSV(strBuffer);
                Location locTrue = smp.location();
                double dist2D = Location::distance2D(locTrue, *poseEst);
                this->mCurrentError = dist2D;
                std::cout<< "Filt, ts=" << smp.timestamp() << ",poseEst=" << *poseEst
                <<  ",locTrue=" << locTrue << ",d2D=" << dist2D <<std::endl;
                if(wasReset){
                    if(mResultDir.size()>0){
                        sstream << locTrue << "," << *poseEst << std::endl;
                        if(savesStates){
                            std::stringstream ss;
                            std::shared_ptr<std::vector<State>> states = status->states();
                            for(State state: *states){
                                ss << state << std::endl;
                            }
                            std::string filepath = mResultDir+"/states_"+std::to_string(beacons.timestamp())+".csv";
                            DataLogger::getInstance()->log(filepath, ss.str());
                        }
                    }
                }
            }catch (std::invalid_argument e){
                std::cout << "invalid sampleCSV was found." << std::endl;
            }
            count_putBeacons ++;
        }else if(DataUtils::csvCheckSensorType(strBuffer, "Reset")){
            Pose poseReset = DataUtils::parseResetPoseCSV(strBuffer);
            std::cout << "Created ResetPose=" << poseReset << std::endl;
            //mStreamLocalizer->resetStatus(poseReset);
            mStreamLocalizer->resetStatus(poseReset, stdevPose_);
            Status* status = mStreamLocalizer->getStatus();
            std::shared_ptr<Pose> poseEst = status->meanPose();
            if(poseEst){
                std::cout << "After reset, poseEst=" << *poseEst << std::endl;
                wasReset = true;
            }
        }
    }
    
    void VirtualDevice::run(){
        char*cwd = getcwd(NULL, 0);
        std::cout << cwd << std::endl;
        free(cwd);
        
        std::ifstream ifs(mCsvPath);
        if(! ifs.is_open()){
            std::cout << mCsvPath << " is not open." << std::endl;
        }
        std::string strBuffer;
        
        clock_t start = clock();
        while(std::getline(ifs, strBuffer)){
            processLine(strBuffer);
        }
        clock_t end = clock();
        std::cout << "end-start=" << end-start << std::endl;
        std::cout << "average=" << (end-start)*1.0f/count_putBeacons/CLOCKS_PER_SEC << std::endl;
        close();
    }
    
    void VirtualDevice::close(){
        if(mResultDir.size()>0){
            std::string path = mResultDir + "/result.csv";
            std::ofstream ofs(path);
            ofs << sstream.str();
            ofs.close();
        }else{
            std::cout << "Results were not saved."  << std::endl;
        }
    }
    
    
}