/*******************************************************************************
 * Copyright (c) 2014, 2016  IBM Corporation and others
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

#include "NavCogLogPlayer.hpp"

namespace loc{

    void NavCogLogPlayer::run(){
        std::ifstream ifs(mFilePath);
        if(! ifs.is_open()){
            std::cout << mFilePath << " is not open." << std::endl;
        }
        std::string strBuffer;
        
        clock_t start = clock();
        while(std::getline(ifs, strBuffer)){
            processLine(strBuffer);
        }
        clock_t end = clock();
    }
    
    void NavCogLogPlayer::processLine(std::string strBuffer){
        struct tm tm; int ms;
        memset(&tm, 0, sizeof(struct tm));
        sscanf(strptime(strBuffer.c_str(), "%Y-%m-%d %H:%M:%S", &tm), ".%d", &ms);
        long time_stamp = mktime(&tm)*1000+ms;

        std::vector<std::string> v;
        boost::algorithm::split( v, strBuffer, boost::algorithm::is_space() );
        /*
        namespace pt = boost::posix_time;
        namespace gg = boost::gregorian;
        
        std::string ts = v.at(0) + " " + v.at(1);
        typedef boost::date_time::c_local_adjustor<pt::ptime> local_adj;
        auto epoch = local_adj::utc_to_local(pt::ptime(gg::date(1970, 1, 1)));
        
        auto date_time = pt::time_from_string(ts);
        auto diff = date_time - epoch;
        long time_stamp = diff.total_milliseconds();
        */
        
        //boost::algorithm::split( v, strBuffer, boost::is_any_of(" :"));
        //long time_stamp = (std::stoi(v.at(1))*3600+std::stoi(v.at(2))*60+std::stof(v.at(3)))*1000;
        
        boost::algorithm::split( v, strBuffer, boost::algorithm::is_space() );
        // v.at(2) is not used because it is NavCog[???:???]
        
        std::stringstream ss;
        ss << time_stamp;
        ss << ",";
        ss << v.at(3);
        for(int i=4; i<v.size(); i++){
            ss << "," << v.at(i);
        }
        
        std::string strBuffer2 = ss.str();
        
        //std::cout << "strBuffer2=" << strBuffer2 << std::endl;
        
        if(DataUtils::csvCheckBeacons(strBuffer2)){
            boost::algorithm::split(v, strBuffer2, boost::is_any_of(","));
            std::stringstream ss2;
            ss2 << v.at(0) << "," << v.at(1) << "," << "0,0,0,0"; // Fill artificial location data
            for(int i=2; i<v.size(); i++){
                ss2 << "," << v.at(i);
            }
            Beacons bs = DataUtils::parseBeaconsCSV(ss2.str());
            mFuncBeacons(bs);
        }
        if(DataUtils::csvCheckAcceleration(strBuffer2)){
            Acceleration acc = DataUtils::parseAccelerationCSV(strBuffer2);
            mFuncAcc(acc);
        }
        if(DataUtils::csvCheckAttitude(strBuffer2)){
            Attitude att = DataUtils::parseAttitudeCSV(strBuffer2);
            mFuncAtt(att);
        }
        if(DataUtils::csvCheckSensorType(strBuffer2, "Reset")){
            Pose poseReset = DataUtils::parseResetPoseCSV(strBuffer2);
            mFuncReset(poseReset);
        }
    }
    
    
}
