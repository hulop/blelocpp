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

#include "LogUtil.hpp"
#include "DataUtils.hpp"
#include <boost/format.hpp>
#include <string>
#include <sstream>
#include <iostream>

using namespace std;
using namespace loc;

string LogUtil::toString(const Beacons& beacons, const std::uint32_t version) {
    if(version==0){
        std::stringbuf buffer;
        std::ostream os (&buffer);
        os << "Beacon," << beacons.size() << ",";
        for(const Beacon&b: beacons) {
            os << b.majorId() << "," << b.minorId() << "," << b.rssi() << ",";
        }
        os << beacons.timestamp();
        return buffer.str();
    }else{
        std::stringbuf buffer;
        std::ostream os (&buffer);
        os << "Beacon," << beacons.size() << ",";
        for(const Beacon&b: beacons) {
            os << b.uuid() << "-" << b.majorId() << "-" << b.minorId() << "," << b.rssi() << ",";
        }
        os << beacons.timestamp();
        return buffer.str();
    }
}

string LogUtil::toString(const Acceleration& acc) {
    std::stringbuf buffer;
    std::ostream os (&buffer);
    os << "Acc," << acc.ax() << "," << acc.ay() << "," << acc.az() << "," << acc.timestamp();
    return buffer.str();
}

string LogUtil::toString(const Attitude& att) {
    std::stringbuf buffer;
    std::ostream os (&buffer);
    os << "Motion," << att.pitch() << "," << att.roll() << "," << att.yaw() << "," << att.timestamp();
    return buffer.str();
}

string LogUtil::toString(const Altimeter& alt) {
    std::stringbuf buffer;
    std::ostream os (&buffer);
    os << "Altimeter," << alt.relativeAltitude() << "," << alt.pressure() << "," << alt.timestamp();
    return buffer.str();
}

Beacons LogUtil::toBeacons(std::string str){
    return DataUtils::parseLogBeaconsCSV(str);
}

Acceleration LogUtil::toAcceleration(std::string str){
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(","));
    if(values.at(0).compare("Acc")!=0){
        BOOST_THROW_EXCEPTION(LocException("Log Acc is not correct. string="+str));
    }
    long timestamp = stol(values.at(4));
    Acceleration acc(timestamp,
                     stod(values.at(1)),
                     stod(values.at(2)),
                     stod(values.at(3))
                     );
    return acc;
}

Attitude LogUtil::toAttitude(std::string str){
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(","));
    if(values.at(0).compare("Motion")!=0){
        BOOST_THROW_EXCEPTION(LocException("Log Motion is not correct. string="+str));
    }
    // "Motion",pitch,roll,yaw,timestamp
    double pitch = stod(values.at(1));
    double roll = stod(values.at(2));
    double yaw = stod(values.at(3));
    long timestamp = stol(values.at(4));
    Attitude att(timestamp, pitch, roll, yaw);// Attitude(timestamp, pitch, roll, yaw)
    return att;
}


Altimeter LogUtil::toAltimeter(const std::string& str){
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(","));
    if(values.at(0).compare("Altimeter")!=0){
        BOOST_THROW_EXCEPTION(LocException("Log Altimeter is not correct. string="+str));
    }
    // "Altimeter",relativeAltitude,pressure,timestamp
    long timestamp = stol(values.at(3));
    double relAlt = stod(values.at(1));
    double pressure = stod(values.at(2));
    Altimeter alt(timestamp,relAlt,pressure);
    return alt;
}

//Heading
string LogUtil::toString(const Heading& heading){
    //"Heading",magneticHeading,trueHeading,headingAccuracy,mx,my,mz,timestamp
    std::stringstream ss;
    ss << "Heading," << heading.magneticHeading() << "," << heading.trueHeading() << ","
    << heading.headingAccuracy() << ","
    << heading.x() << "," << heading.y() << "," << heading.z() << ","
    << heading.timestamp();
    return ss.str();
}
Heading LogUtil::toHeading(const std::string& str){
    std::vector<std::string> values;
    boost::split(values, str, boost::is_any_of(","));
    if(values.at(0).compare("Heading")!=0){
        BOOST_THROW_EXCEPTION(LocException("Log Heading is not correct. string="+str));
    }
    //"Heading",magneticHeading,trueHeading,headingAccuracy,mx,my,mz,timestamp
    long timestamp = stol(values.back());
    double magneticHeading = stod(values.at(1));
    double trueHeading = stod(values.at(2));
    double headingAccuracy = stod(values.at(3));
    if(8<=values.size()){
        double mx = stod(values.at(4));
        double my = stod(values.at(5));
        double mz = stod(values.at(6));
        Heading head(timestamp, magneticHeading, trueHeading, headingAccuracy, mx, my, mz);
        return head;
    }else{
        Heading head(timestamp, magneticHeading, trueHeading, headingAccuracy);
        return head;
    }
}

