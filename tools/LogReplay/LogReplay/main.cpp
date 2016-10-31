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
#include <string>
#include <sstream>
#include <fstream>


#include "bleloc.h"
#include <getopt.h>
#include <boost/program_options.hpp>
#include "NavCogLogPlayer.hpp"
#include "DataLogger.hpp"
#include "ExtendedDataUtils.hpp"
#include "StreamParticleFilterBuilder.hpp"

struct Option{
    std::string trainFilePath = "";
    std::string logFilePath = "";
    std::string beaconFilePath = "";
    std::string mapFilePath = "";
    std::string outputFilePath = "";
    bool shortCSV = false;
    bool jsonSample = false;
    float unit = 1.0;
    bool oneDPDR = false;
    float starty = 0;
    float endy = 0;
    float alphaWeaken = 0.3;
    bool randomWalker = false;
    bool oneshot = false;
    bool considerBias = false;
    std::string trainedModelPath = "";
    std::string directoryLog = "";
    double gridSize = 0.5;
    double meanRssiBias = 0.0;
    double stdRssiBias = 2.0;
    double minRssiBias = -10;
    double maxRssiBias = 10;
    double diffusionRssiBias = 0.2;
    double stdX = 1.0;
    double stdY = 1.0;
    double tDistribution = 0;
    
    void print(){
        std::cout << "------------------------------------" << std::endl;
        std::cout << " trainFilePath  =" << trainFilePath << std::endl;
        std::cout << " shortCSV       =" << (shortCSV?"true":"false") << std::endl;
        std::cout << " jsonSample     =" << (jsonSample?"true":"false") << std::endl;
        std::cout << " unit scale     =" << unit << std::endl;
        std::cout << " beaconFilePath =" << beaconFilePath << std::endl;
        std::cout << " mapFilePath    =" << mapFilePath << std::endl;
        std::cout << " logFilePath    =" << logFilePath << std::endl;
        std::cout << " outputFilePath =" << outputFilePath << std::endl;
        std::cout << " oneDPDR        =" << (oneDPDR?"true":"false") << " (" << starty << "->" << endy << ")" << std::endl;
        std::cout << " alphaWeaken    =" << alphaWeaken << std::endl;
        std::cout << " randomWalker   =" << (randomWalker?"true":"false") << std::endl;
        std::cout << " modelPath      =" << trainedModelPath << std::endl;
        std::cout << " oneshot        =" << (oneshot?"true":"false") << std::endl;
        std::cout << " considerBias   =" << (considerBias?"true":"false") << std::endl;
        std::cout << " directoryLog   =" << directoryLog << std::endl;
        std::cout << " gridSize       =" << gridSize << std::endl;
        std::cout << " minRssiBias    =" << minRssiBias << std::endl;
        std::cout << " maxRssiBias    =" << maxRssiBias << std::endl;
        std::cout << " meanRssiBias   =" << meanRssiBias << std::endl;
        std::cout << " stdRssiBias    =" << stdRssiBias << std::endl;
        std::cout << " diffusionRssiBias = " << diffusionRssiBias << std::endl;
        std::cout << " tDistribution  =" << tDistribution << std::endl;
        std::cout << " stdX    =" << stdX << std::endl;
        std::cout << " stdY    =" << stdY << std::endl;
        std::cout << "------------------------------------" << std::endl;
    }
};

std::string lastComponent(char *cstr) {
    std::string str(cstr);
    std::size_t found = str.find_last_of("/");
    return str.substr(found+1);
}

void printHelp(std::string command){
    std::cout << "Options for NavCog log play" << std::endl;
    std::cout << " -h                   show this help" << std::endl;
    std::cout << " -t trainingDataFile  set training data file (long csv format)" << std::endl;
    std::cout << " -s                   indicates <trainingDataFile> is in short csv format (3-feet unit)" << std::endl;
    std::cout << " -j                   indicates <trainingDataFile> is in json sample format" << std::endl;
    std::cout << " -b beaconDataFile    set beacon data file" << std::endl;
    std::cout << " -f                   indicates <beaconDataFile> is in feet unit" << std::endl;
    std::cout << " -m [#,#,#,#,#,]path  set map image file (PNG) for floor, ppmx, ppmy, originx, originy, and path respectively, if option is not provided 0,8,-8,1000,1000 are used." << std::endl;
    std::cout << " -l logFile           set NavCog log file" << std::endl;
    std::cout << " -1 starty,endy       set 1D-PDR mode and the start/end point. specify like -1 0,9 in feet" << std::endl;
    std::cout << " -o outputFile        set output file" << std::endl;
    std::cout << " -a <float>           set alphaWeaken value" << std::endl;
    std::cout << " -r                   use random walker instead pdr" << std::endl;
    std::cout << " -p modelFile         set the name of saved model file" << std::endl;
    std::cout << " -n                   set oneshot mode" << std::endl;
    std::cout << " -c                   consider bias for oneshot" << std::endl;
    std::cout << " -d                   set directory to output log play details" <<std::endl;
    std::cout << " -g                   set grid size to evaluate observation model" << std::endl;
    std::cout << " --minRssiBias        set minimum value of rssi bias" << std::endl;
    std::cout << " --maxRssiBias        set maximum value of rssi bias" << std::endl;
    std::cout << " --meanRssiBias       set mean of rssi bias at initialization" << std::endl;
    std::cout << " --stdRssiBias        set standard deviation of rssi bias at initialization" << std::endl;
    std::cout << " --diffusionRssiBias  set diffusion rssi bias" << std::endl;
    std::cout << " --stdX <float>       set standard deviation of x used in initialization and mcmc sampling" << std::endl;
    std::cout << " --stdY <float>       set standard deviation of y used in initialization and mcmc sampling" << std::endl;
    std::cout << " --students-t <float> set beacon rssi distribution as student's t distribution" << std::endl;
    std::cout << std::endl;
    std::cout << "Example" << std::endl;
    std::cout << "$ " << command << " -t train.txt -b beacon.csv -m map.png -l navcog.log -o out.txt" << std::endl;
    std::cout << std::endl;
}

Option parseArguments(int argc,char *argv[]){
    Option opt;
    
    int c = 0;
    int option_index = 0;
    struct option long_options[] = {
        {"minRssiBias",     required_argument, NULL,  0 },
        {"maxRssiBias",     required_argument, NULL,  0 },
        {"meanRssiBias",     required_argument, NULL,  0 },
        {"stdRssiBias",     required_argument, NULL,  0 },
        {"diffusionRssiBias",     required_argument, NULL,  0 },
        {"stdX",            required_argument, NULL,  0 },
        {"stdY",            required_argument, NULL,  0 },
        {"tDistribution",   required_argument, NULL,  0 },
        {0,         0,                 0,  0 }
    };
//while ((c = getopt (argc, argv, "shft:b:l:o:m:1:a:rp:njcd:g:")) != -1)
    while ((c = getopt_long(argc, argv, "shft:b:l:o:m:1:a:rp:njcd:g:", long_options, &option_index )) != -1)
        switch (c)
    {
        case 0:
            if (strcmp(long_options[option_index].name, "minRssiBias") == 0){
                opt.minRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "maxRssiBias") == 0){
                opt.maxRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "meanRssiBias") == 0){
                opt.meanRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "stdRssiBias") == 0){
                opt.stdRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "diffusionRssiBias") == 0){
                opt.diffusionRssiBias = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "stdX") == 0){
                opt.stdX = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "stdY") == 0){
                opt.stdY = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "tDistribution") == 0) {
                opt.tDistribution = atof(optarg);
            }
            break;
        case 'h':
            printHelp(lastComponent(argv[0]));
            abort();
        case 't':
            opt.trainFilePath.assign(optarg);
            break;
        case '1':
            opt.oneDPDR = true;
            sscanf(optarg, "%f,%f",&(opt.starty),&(opt.endy));
            break;
        case 's':
            opt.shortCSV = true;
            break;
        case 'j':
            opt.jsonSample = true;
            break;
        case 'f':
            opt.unit = 0.3048;
            break;
        case 'b':
            opt.beaconFilePath.assign(optarg);
            break;
        case 'l':
            opt.logFilePath.assign(optarg);
            break;
        case 'o':
            opt.outputFilePath.assign(optarg);
            break;
        case 'm':
            opt.mapFilePath.assign(optarg);
            break;
        case 'a':
            sscanf(optarg, "%f", &(opt.alphaWeaken));
            break;
        case 'r':
            opt.randomWalker = true;
            break;
        case 'p':
            opt.trainedModelPath.assign(optarg);
            break;
        case 'n':
            opt.oneshot = true;
            break;
        case 'c':
            opt.considerBias = true;
            break;
        case 'd':
            opt.directoryLog.assign(optarg);
            break;
        case 'g':
            sscanf(optarg, "%lf", &(opt.gridSize));
            break;
        default:
            abort();
    }
    
    return opt;
}

struct UserData{
    std::stringstream ss;
};

struct Location{
    double x;
    double y;
    double z;
    int floor;
};

static double reached = NAN;
static struct Location groundTruth = {NAN,NAN,NAN,INT_MAX};

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    UserData* ud = (UserData*) userData;
    long timestamp = pStatus->timestamp();
    auto meanPose = pStatus->meanPose();
    
    if (!isnan(reached)) {
        std::cout << timestamp << "," << *meanPose << "," << reached << std::endl;
        ud->ss << timestamp << "," << *meanPose << "," << reached << std::endl;
    } else if (!isnan(groundTruth.x) && !isnan(groundTruth.y) && !isnan(groundTruth.z) && groundTruth.floor != INT_MAX) {
        std::cout << timestamp << "," << *meanPose << "," << groundTruth.x << "," << groundTruth.y << "," << groundTruth.z << "," << groundTruth.floor << std::endl;
        ud->ss << timestamp << "," << *meanPose << "," << groundTruth.x << "," << groundTruth.y << "," << groundTruth.z << "," << groundTruth.floor << std::endl;
    } else {
        std::cout << timestamp << "," << *meanPose << std::endl;
        ud->ss << timestamp << "," << *meanPose << std::endl;
    }
    
    // TODO calculate error here with the ground truth. (static variable)
    // meanPose->y() is the y value.
}

loc::Beacons convertBLEBeaconsToDummyBeacons(const loc::BLEBeacons& bleBeacons){
    loc::Beacons beacons;
    for(const auto& ble: bleBeacons){
        double rssi = -10; //dummy value
        loc::Beacon b(ble.major(), ble.minor(), rssi);
        beacons.push_back(b);
    }
    return beacons;
}

std::vector<loc::Location> convertSamplesToGridLocations(const loc::Samples& samples, double dx, double dy){
    std::vector<double> xvec, yvec, zvec, fvec;
    for(const auto& s:samples){
        const auto& loc = s.location();
        xvec.push_back(loc.x());
        yvec.push_back(loc.y());
        zvec.push_back(loc.z());
        fvec.push_back(loc.floor());
    }
    double xmin = *std::min_element(xvec.begin(),xvec.end());
    double xmax = *std::max_element(xvec.begin(),xvec.end());
    double ymin = *std::min_element(yvec.begin(),yvec.end());
    double ymax = *std::max_element(yvec.begin(),yvec.end());
    double zmin = *std::min_element(zvec.begin(),zvec.end());
    double zmax = *std::max_element(zvec.begin(),zvec.end());
    double fmin = *std::min_element(fvec.begin(),fvec.end());
    double fmax = *std::max_element(fvec.begin(),fvec.end());
    
    double z=zmin, f=fmin;
    std::vector<loc::Location> locs;
    for(double x=std::floor(xmin); x<=xmax+1 ; x+=dx){
        for(double y=std::floor(ymin); y<=ymax+1 ; y+=dy){
            loc::Location loc(x,y,z,f);
            locs.push_back(loc);
        }
    }
    return locs;
}

int main(int argc,char *argv[]){
    
    if (argc <= 1) {
        printHelp(lastComponent(argv[0]));
        return 0;
    }
    
    using namespace loc;
    
    std::cout<<"Log play start"<<std::endl;
    
    Option opt = parseArguments(argc, argv);
    opt.print();
    
    loc::StreamParticleFilterBuilder builder;
    builder.usesObservationDependentInitializer = false;
    builder.mixProbability = 0.0;
    builder.trainDataPath(opt.trainFilePath);
    builder.shortCSV = opt.shortCSV;
    builder.jsonSample = opt.jsonSample;
    builder.unit = opt.unit;
    builder.alphaWeaken = opt.alphaWeaken;
    builder.beaconDataPath(opt.beaconFilePath);
    builder.mapDataPath(opt.mapFilePath);
    builder.randomWalker = opt.randomWalker;
    builder.trainedModelPath = opt.trainedModelPath;
    builder.considerBias = opt.considerBias;
    builder.minRssiBias = opt.minRssiBias;
    builder.maxRssiBias = opt.maxRssiBias;
    builder.meanRssiBias = opt.meanRssiBias;
    builder.stdRssiBias = opt.stdRssiBias;
    builder.diffusionRssiBias = opt.diffusionRssiBias;
    builder.poseProperty_stdX = opt.stdX;
    builder.poseProperty_stdY = opt.stdY;
    builder.tDistribution = opt.tDistribution;
    std::shared_ptr<loc::StreamLocalizer> localizer = builder.build();
        
    UserData userData;
    localizer->updateHandler(functionCalledWhenUpdated, &userData);
    
    if(opt.directoryLog!=""){
        DataLogger::createInstance(opt.directoryLog);
    }
    // Predict rssis at grid positions generated from sampling ploints.
    if(opt.directoryLog!=""){
        auto ds = builder.dataStore();
        auto beacons = convertBLEBeaconsToDummyBeacons(ds->getBLEBeacons());
        auto locs = convertSamplesToGridLocations(ds->getSamples(), opt.gridSize, opt.gridSize);
        
        picojson::array jarray;
        for(const loc::Location& loc : locs){
            State s(loc);
            auto obsModel = builder.obsModel();
            std::map<long,std::vector<double>> idRssiStats = obsModel->predict(s, beacons);
            auto jobj = ExtendedDataUtils::predictionDataToJSONObject(s, beacons, idRssiStats);
            jarray.push_back(picojson::value(jobj));
        }
        picojson::object jobj;
        jobj.insert(std::make_pair("data", picojson::value(jarray)));
        if(DataLogger::getInstance()){
            std::stringstream ss;
            ss << "grid_predictions.json";
            DataLogger::getInstance()->log(ss.str(), picojson::value(jobj).serialize());
        }
    }
    
    if (opt.oneDPDR) {
        loc::Pose pose;
        float orientation = atan2(opt.endy-opt.starty, 0);
        pose.x(0).y(opt.starty*0.3048).z(0).floor(0).orientation(orientation);
        loc::Pose stdevPose;
        stdevPose.x(0.25).y(0.25).orientation(1.0/180.0*M_PI);
        localizer->resetStatus(pose, stdevPose);
    }
    
    loc::NavCogLogPlayer logPlayer;
    logPlayer.filePath(opt.logFilePath);
    logPlayer.functionCalledWhenBeaconsUpdated([&](Beacons beacons){
        if (opt.randomWalker) {
            localizer->resetStatus();
        }
        if (opt.oneshot) {
            localizer->resetStatus(beacons);
            functionCalledWhenUpdated((void*)&userData, localizer->getStatus());
        } else {
            localizer->putBeacons(beacons);
        }
        
        if(opt.directoryLog!=""){
            long ts = beacons.timestamp();
            auto states = localizer->getStatus()->states();
            picojson::array jarray;
            for(const State& s : *states){
                auto obsModel = builder.obsModel();
                std::map<long,std::vector<double>> idRssiStats = obsModel->predict(s, beacons);
                auto jobj = ExtendedDataUtils::predictionDataToJSONObject(s, beacons, idRssiStats);
                jarray.push_back(picojson::value(jobj));
            }
            picojson::object jobj;
            jobj.insert(std::make_pair("timestamp", picojson::value((double)ts)));
            jobj.insert(std::make_pair("data", picojson::value(jarray)));
            if(DataLogger::getInstance()){
                std::stringstream ss;
                ss << "prediction_detail_t" << ts << ".json";
                DataLogger::getInstance()->log(ss.str(), picojson::value(jobj).serialize());
            }
        }
    });
    logPlayer.functionCalledWhenAccelerationUpdated([&](Acceleration acc){
        localizer->putAcceleration(acc);
    });
    logPlayer.functionCalledWhenAttitudeUpdated([&](Attitude att){
        if (opt.oneDPDR) {
            att = Attitude(att.timestamp(), 0, 0, 0);
        }
        localizer->putAttitude(att);
    });
    logPlayer.functionCalledWhenReset([&](Pose poseReset){
        //localizer->resetStatus(poseReset);
        // TODO start error calcuration from when this is called
    });
    logPlayer.functionCalledWhenReached([&](long time_stamp, double pos){
        // TODO update ground truth position
        // std::cout << time_stamp << "," << pos*3*opt.unit << ",Reached" << std::endl;
        reached = pos*3*0.3048;
    });
    logPlayer.functionCalledWhenGroundTruth([&](long time_stamp, double x, double y, double z, double floor) {
        groundTruth = {x,y,z,(int)floor};
    });

    if(opt.logFilePath!=""){
        logPlayer.run();
    }
    
    if(opt.outputFilePath!=""){
        std::ofstream ofs(opt.outputFilePath);
        ofs << "timestamp," << Pose::header() << std::endl;
        ofs << userData.ss.str();
    }
    
    std::cout<<"Log play end"<<std::endl;
    return 0;
}