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

#include <iostream>
#include "BasicLocalizer.hpp"
#include "MathUtils.hpp"
#include "DataUtils.hpp"
#include "LogUtil.hpp"
#include <getopt.h>
#include <boost/program_options.hpp>

using namespace loc;


typedef struct {
    std::string mapPath = "";
    std::string testPath = "";
    std::string outputPath = "";
    double meanRssiBias = 0.0;
    double minRssiBias = -10;
    double maxRssiBias = 10;
    NormalFunction normFunc = NORMAL;
    double tDistNu = 3;
    int nSmooth = 3;
    int nStates = 1000;
    SmoothType smoothType = SMOOTH_LOCATION;
    bool findRssiBias = false;
    LocalizeMode localizeMode = ONESHOT;
    double walkDetectSigmaThreshold = 0.6;
    bool usesReset = false;
} Option;

void printHelp() {
    std::cout << "Options for Basic Localizer" << std::endl;
    std::cout << " -h                  show this help" << std::endl;
    std::cout << " -m mapfile          set map data file" << std::endl;
    std::cout << " -t testfile         set test csv data file" << std::endl;
    std::cout << " -o output           set output file" << std::endl;
    std::cout << " -n                  use normal distribution" << std::endl;
    std::cout << " --minRssiBias       set minimum value of rssi bias" << std::endl;
    std::cout << " --maxRssiBias       set maximum value of rssi bias" << std::endl;
    std::cout << " --meanRssiBias      set mean of rssi bias at initialization" << std::endl;
    std::cout << " --nSmooth           set nSmooth" << std::endl;
    std::cout << " -r                  set beacon rssi smooth (default location smooth)" << std::endl;
    std::cout << " -s <double>         use student's t distribution and set nu value" << std::endl;
    std::cout << " -f                  find rssiBias" << std::endl;
    std::cout << " --lm <string>       set localization mode [ONESHOT,RANDOM_WALK_ACC,RANDOM_WALK_ACC_ATT,WEAK_POSE_RANDOM_WALKER]" << std::endl;
    std::cout << " --wc                use wheelchair mode set" << std::endl;
    std::cout << " --reset             use reset in log" << std::endl;
}

Option parseArguments(int argc, char *argv[]){
    Option opt;
    
    int c = 0;
    int option_index = 0;
    struct option long_options[] = {
        {"minRssiBias",     required_argument, NULL,  0 },
        {"maxRssiBias",     required_argument, NULL,  0 },
        {"meanRssiBias",    required_argument, NULL,  0 },
        {"nSmooth",    required_argument, NULL,  0 },
        {"lm",         required_argument, NULL,  0 },
        {"wc",         no_argument, NULL, 0},
        {"reset",      no_argument, NULL, 0},
        //{"stdY",            required_argument, NULL,  0 },
        {0,         0,                 0,  0 }
    };

    while ((c = getopt_long(argc, argv, "m:t:ns:ho:rf", long_options, &option_index )) != -1)
        switch (c)
    {
        case 0:
            //if (strcmp(long_options[option_index].name, "stdY") == 0){
            //opt.stdY = atof(optarg);
            //}
            if (strcmp(long_options[option_index].name, "minRssiBias") == 0){
                opt.minRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "maxRssiBias") == 0){
                opt.maxRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "meanRssiBias") == 0){
                opt.meanRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "nSmooth") == 0){
                opt.nSmooth = atoi(optarg);
            }
            if (strcmp(long_options[option_index].name, "lm") == 0){
                if(strcmp(optarg, "ONESHOT") == 0){
                    opt.localizeMode = ONESHOT;
                }else if(strcmp(optarg, "RANDOM_WALK_ACC") == 0){
                    opt.localizeMode = RANDOM_WALK_ACC;
                }else if(strcmp(optarg, "RANDOM_WALK_ACC_ATT") == 0){
                    opt.localizeMode = RANDOM_WALK_ACC_ATT;
                }else if(strcmp(optarg, "WEAK_POSE_RANDOM_WALKER") == 0){
                    opt.localizeMode = WEAK_POSE_RANDOM_WALKER;
                }else{
                    std::cerr << "Unknown localization mode: " << optarg << std::endl;
                    abort();
                }
            }
            if (strcmp(long_options[option_index].name, "wc") == 0){
                opt.walkDetectSigmaThreshold = 0.1;
            }
            if (strcmp(long_options[option_index].name, "reset") == 0){
                opt.usesReset = true;
            }
            break;
        case 'h':
            printHelp();
            abort();
        case 'm':
            opt.mapPath.assign(optarg);
            break;
        case 't':
            opt.testPath.assign(optarg);
            break;
        case 'n':
            opt.normFunc = NORMAL;
            break;
        case 'o':
            opt.outputPath.assign(optarg);
            break;
        case 's':
            opt.normFunc = TDIST;
            sscanf(optarg, "%lf", &(opt.tDistNu));
            break;
        case 'r':
            opt.smoothType = SMOOTH_RSSI;
            break;
        case 'f':
            opt.findRssiBias = true;
            break;
        default:
            abort();
    }
    return opt;
}

typedef struct {
    Option *opt;
    std::ostream *out;
    std::vector<loc::Status*> status_list;
    LatLngConverter::Ptr latLngConverter;
    Pose recentPose;
} MyData;

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    MyData *ud = (MyData*)userData;
    if (ud->opt->findRssiBias) {
        ud->status_list.insert(ud->status_list.end(), pStatus);
    } else {
        if(pStatus->step()==Status::FILTERING_WITH_RESAMPLING){
            auto meanLoc = ud->latLngConverter->localToGlobal(*pStatus->meanLocation());
            *ud->out << meanLoc << std::endl;
            ud->recentPose = *pStatus->meanPose();
        }
        int i=0;
        for(const State& s: *pStatus->states()) {
            auto gs = ud->latLngConverter->localToGlobal(s);
            assert(gs.lat() != 0);
            assert(gs.lng() != 0);
        }
    }
}

int main(int argc, char * argv[]) {
    // insert code here...
    if (argc <= 1) {
        printHelp();
        return 0;
    }
    
    Option opt = parseArguments(argc, argv);
    
    MyData ud;
    ud.opt = &opt;
    if (opt.outputPath.length() > 0) {
        ud.out = new std::ofstream(opt.outputPath);
        if (ud.out->fail()) {
            std::cerr << "output file is unable to write: " << opt.outputPath << std::endl;
            return -1;
        }
    } else {
        ud.out = &std::cout;
    }

    BasicLocalizer localizer;
    localizer.localizeMode = opt.localizeMode;
    
    localizer.nSmooth = opt.nSmooth;
    localizer.smoothType = opt.smoothType;
    localizer.nStates = opt.nStates;
    
    localizer.updateHandler(functionCalledWhenUpdated, &ud);
    localizer.walkDetectSigmaThreshold = opt.walkDetectSigmaThreshold;
    // Some parameters must be set before calling setModel function. 
    localizer.setModel(opt.mapPath, "./");

    localizer.normalFunction(opt.normFunc, opt.tDistNu);
    localizer.meanRssiBias(opt.meanRssiBias);
    localizer.minRssiBias(opt.minRssiBias);
    localizer.maxRssiBias(opt.maxRssiBias);
    
    ud.latLngConverter = localizer.latLngConverter();

    if (opt.findRssiBias) {
        double step = (opt.maxRssiBias - opt.minRssiBias)/20;
        for(double rssiBias = opt.minRssiBias; rssiBias < opt.maxRssiBias; rssiBias += step) {
            localizer.meanRssiBias(rssiBias);
            localizer.minRssiBias(rssiBias-step);
            localizer.maxRssiBias(rssiBias+step);
            ud.status_list.empty();
            
            std::ifstream ifs(opt.testPath);
            std::string str;
            if (ifs.fail())
            {
                std::cerr << "test file is unable to read: " << opt.testPath << std::endl;
                return -1;
            }
            while (getline(ifs, str))
            {
                try {
                    std::list<std::string> stringList;
                    std::string delim (" ");
                    boost::split(stringList, str, boost::is_any_of(delim));
                    
                    for(auto iter = stringList.begin(); iter != stringList.end(); iter++) {
                        if (iter->compare(0, 6, "Beacon") == 0) {
                            Beacons beacons = DataUtils::parseBeaconsCSV(*iter);
                            localizer.putBeacons(beacons);
                            break;
                        }
                    }
                    
                } catch (std::invalid_argument e){
                    std::cerr << e.what() << std::endl;
                }
            }
            
            for(loc::Status *st: ud.status_list) {
                loc::Location stdev = loc::Location::standardDeviation(*st->states());
                std::cout << rssiBias << "," << stdev << std::endl;
            }
        }
        
    } else {
        std::ifstream ifs(opt.testPath);
        std::string str;
        if (ifs.fail())
        {
            std::cerr << "test file is unable to read: " << opt.testPath << std::endl;
            return -1;
        }
        
        while (getline(ifs, str))
        {
            try {
                std::vector<std::string> v;
                boost::split(v, str, boost::is_any_of(" "));
                if(v.size() > 3){
                    std::string logString = v.at(3);
                    // Parsing beacons values
                    if (logString.compare(0, 6, "Beacon") == 0) {
                        Beacons beacons = LogUtil::toBeacons(logString);
                        localizer.putBeacons(beacons);
                    }
                    // Parsing acceleration values
                    if (logString.compare(0, 3, "Acc") == 0) {
                        Acceleration acc = LogUtil::toAcceleration(logString);
                        localizer.putAcceleration(acc);
                    }
                    // Parsing motion values
                    if (logString.compare(0, 6, "Motion") == 0) {
                        Attitude att = LogUtil::toAttitude(logString);
                        localizer.putAttitude(att);
                    }
                    if (opt.usesReset && logString.compare(0, 5, "Reset") == 0) {
                        // "Reset",lat,lng,floor,heading,timestamp
                        std::vector<std::string> values;
                        boost::split(values, logString, boost::is_any_of(","));
                        long timestamp = stol(values.at(5));
                        double lat = stod(values.at(1));
                        double lng = stod(values.at(2));
                        double floor = stod(values.at(3));
                        double heading = stod(values.at(4));
                        std::cout << "LogReplay:" << timestamp << ",Reset,";
                        std::cout << std::setprecision(10) << lat <<"," <<lng;
                        std::cout <<"," <<floor <<"," << heading << std::endl;
                        
                        Location loc;
                        GlobalState<Location> global(loc);
                        global.lat(lat);
                        global.lng(lng);
                        loc = ud.latLngConverter->globalToLocal(global);
                        loc.floor(floor);
                        
                        auto anchor = ud.latLngConverter->anchor();
                        double localHeading = ( heading - anchor.rotate )/180*M_PI;
                        double xH = sin(localHeading);
                        double yH = cos(localHeading);
                        double orientation = atan2(yH,xH);
                        
                        loc::Pose newPose(loc);
                        newPose.orientation(orientation);
                        
                        localizer.resetStatus(newPose);
                    }
                    if (logString.compare(0, 6, "Marker") == 0){
                        // "Marker",lat,lng,floor,timestamp
                        std::vector<std::string> values;
                        boost::split(values, logString, boost::is_any_of(","));
                        double lat = stod(values.at(1));
                        double lng = stod(values.at(2));
                        double floor = stod(values.at(3));
                        long timestamp = stol(values.at(4));

                        Location markerLoc;
                        GlobalState<Location> global(markerLoc);
                        global.lat(lat);
                        global.lng(lng);
                        global.floor(floor);
                        markerLoc = ud.latLngConverter->globalToLocal(global);
                        
                        auto recentPose = ud.recentPose;
                        
                        std::cout << "LogReplay:" << timestamp << ",Marker,";
                        std::cout << std::setprecision(10) << lat << "," << lng;
                        std::cout << "," << floor;
                        std::cout << ",d2D=" << Location::distance2D(markerLoc, recentPose)
                        << ",dFloor=" << Location::floorDifference(markerLoc, recentPose)
                        << std::endl;
                    }
                    if (logString.compare(0, 9,"Altimeter") == 0){
                        // pass
                    }
                    if (logString.compare(0, 7,"Heading") == 0){
                        // pass
                    }
                }
            } catch (std::invalid_argument e){
                std::cerr << e.what() << std::endl;
                std::cerr << "error in parse log file" << std::endl;
            }
        }
    }
    
    return 0;
}
