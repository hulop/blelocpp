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
#include "StreamParticleFilterBUilder.hpp"

struct Option{
    std::string trainFilePath = "";
    std::string logFilePath = "";
    std::string beaconFilePath = "";
    std::string mapFilePath = "";
    std::string outputFilePath = "";
    bool shortCSV = false;
    float unit = 1.0;
    bool oneDPDR = false;
    float starty = 0;
    float endy = 0;
    float alphaWeaken = 0.3;
    bool randomWalker = false;
    std::string trainedModelPath = "";
    
    void print(){
        std::cout << "------------------------------------" << std::endl;
        std::cout << " trainFilePath  =" << trainFilePath << std::endl;
        std::cout << " shortCSV       =" << (shortCSV?"true":"false") << std::endl;
        std::cout << " unit scale     =" << unit << std::endl;
        std::cout << " beaconFilePath =" << beaconFilePath << std::endl;
        std::cout << " mapFilePath    =" << mapFilePath << std::endl;
        std::cout << " logFilePath    =" << logFilePath << std::endl;
        std::cout << " outputFilePath =" << outputFilePath << std::endl;
        std::cout << " oneDPDR        =" << (oneDPDR?"true":"false") << " (" << starty << "->" << endy << ")" << std::endl;
        std::cout << " alphaWeaken    =" << alphaWeaken << std::endl;
        std::cout << " randomWalker   =" << randomWalker << std::endl;
        std::cout << "------------------------------------" << std::endl;
    }
    
    bool isFilled(){
        if(trainFilePath==""){
            return false;
        }
        if(logFilePath==""){
            return false;
        }
        if(beaconFilePath==""){
            return false;
        }
        if(mapFilePath==""){
            return false;
        }
        if(outputFilePath==""){
            return false;
        }
        return true;
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
    std::cout << " -b beaconDataFile    set beacon data file" << std::endl;
    std::cout << " -f                   indicates <beaconDataFile> is in feet unit" << std::endl;
    std::cout << " -m mapImageFile      set map image file (PNG). Map image is treated as 8 pixel per meter." << std::endl;
    std::cout << " -l logFile           set NavCog log file" << std::endl;
    std::cout << " -1 starty,endy       set 1D-PDR mode and the start/end point. specify like -1 0,9 in feet" << std::endl;
    std::cout << " -o outputFile        set output file" << std::endl;
    std::cout << " -a <float>           set alphaWeaken value" << std::endl;
    std::cout << " -r                   use random walker instead pdr" << std::endl;
    std::cout << " -p modelFile         set the name of saved model file" << std::endl;
    std::cout << std::endl;
    std::cout << "Example" << std::endl;
    std::cout << "$ " << command << " -t train.txt -b beacon.csv -m map.png -l navcog.log -o out.txt" << std::endl;
    std::cout << std::endl;
}

Option parseArguments(int argc,char *argv[]){
    Option opt;
    
    int c = 0;
    while ((c = getopt (argc, argv, "shft:b:l:o:m:1:a:rp:")) != -1)
        switch (c)
    {
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
        default:
            abort();
    }
    
    return opt;
}

struct UserData{
    std::stringstream ss;
};

static double reached = NAN;

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    UserData* ud = (UserData*) userData;
    long timestamp = pStatus->timestamp();
    auto meanPose = pStatus->meanPose();
    
    if (!isnan(reached)) {
        std::cout << timestamp << "," << *meanPose << "," << reached << std::endl;
        ud->ss << timestamp << "," << *meanPose << "," << reached << std::endl;
    } else {
        std::cout << timestamp << "," << *meanPose << std::endl;
        ud->ss << timestamp << "," << *meanPose << std::endl;
    }
    
    // TODO calculate error here with the ground truth. (static variable)
    // meanPose->y() is the y value.
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
    builder.unit = opt.unit;
    builder.alphaWeaken = opt.alphaWeaken;
    builder.beaconDataPath(opt.beaconFilePath);
    builder.mapDataPath(opt.mapFilePath);
    builder.randomWalker = opt.randomWalker;
    
    std::shared_ptr<loc::StreamLocalizer> localizer = builder.build();
    if(opt.trainedModelPath!=""){
        builder.saveTrainedModel(opt.trainedModelPath);
    }
    
    UserData userData;
    localizer->updateHandler(functionCalledWhenUpdated, &userData);
    
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
        localizer->putBeacons(beacons);
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
        localizer->resetStatus(poseReset);
        // TODO start error calcuration from when this is called
    });
    logPlayer.functionCalledWhenReached([&](long time_stamp, double pos){
        // TODO update ground truth position
        // std::cout << time_stamp << "," << pos*3*opt.unit << ",Reached" << std::endl;
        reached = pos*3*0.3048;
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