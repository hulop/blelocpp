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
    
    std::string strRequired() const{
        std::stringstream ss;
        ss << "trainFilePath=" << trainFilePath << std::endl;
        ss << "beaconFilePath=" << beaconFilePath << std::endl;
        ss << "mapFilePath=" << mapFilePath << std::endl;
        ss << "logFilePath=" << logFilePath;
        return ss.str();
    }
    
    std::string str() const{
        std::stringstream ss;
        ss << strRequired();
        ss << "outputFilePath=" << outputFilePath;
        return ss.str();
    }
    
    bool isPrepared(){
        if(trainFilePath==""){
            return false;
        }
        if(beaconFilePath==""){
            return false;
        }
        if(mapFilePath==""){
            return false;
        }
        if(logFilePath==""){
            return false;
        }
        /*
        if(outputFilePath==""){
            return false;
        }
        */
        return true;
    }
    
};

void printHelp(){
    std::cout << "Options for NavCog log play" << std::endl;
    std::cout << "-h, Help" << std::endl;
    std::cout << "-t trainingDataFilePath" << std::endl;
    std::cout << "-b beaconDataFilePath" << std::endl;
    std::cout << "-m mapImageFilePath" << std::endl;
    std::cout << "-l logFilePath" << std::endl;
    std::cout << "-o outputFilePath" << std::endl;
}

Option parseArguments(int argc,char *argv[]){
    Option opt;
    
    int c = 0;
    while ((c = getopt (argc, argv, "ht:b:l:o:m:")) != -1)
        switch (c)
    {
        case 'h':
            printHelp();
            exit(0);
        case 't':
            opt.trainFilePath.assign(optarg);
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
        default:
            abort();
    }
    
    if(! opt.isPrepared()){
        std::cerr << "Please input all required options." << std::endl;
        std::cerr << opt.strRequired() << std::endl;
        exit(1);
    }
    return opt;
}

struct UserData{
    std::stringstream ss;
};

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    UserData* ud = (UserData*) userData;
    long timestamp = pStatus->timestamp();
    auto meanPose = pStatus->meanPose();
    std::cout << timestamp << "," << *meanPose << std::endl;
    ud->ss << timestamp << "," << *meanPose << std::endl;
}

int main(int argc,char *argv[]){
    
    using namespace loc;
    
    std::cout<<"Log play start"<<std::endl;
    
    Option opt = parseArguments(argc, argv);
    std::cout << opt.str() << std::endl;
    
    loc::StreamParticleFilterBuilder builder;
    builder.usesObservationDependentInitializer = false;
    builder.mixProbability = 0.0;
    builder.trainDataPath(opt.trainFilePath);
    builder.beaconDataPath(opt.beaconFilePath);
    builder.mapDataPath(opt.mapFilePath);

    std::shared_ptr<loc::StreamLocalizer> localizer = builder.build();
    
    UserData userData;
    localizer->updateHandler(functionCalledWhenUpdated, &userData);
    
    loc::NavCogLogPlayer logPlayer;
    logPlayer.filePath(opt.logFilePath);
    logPlayer.functionCalledWhenBeaconsUpdated([&](Beacons beacons){
        localizer->putBeacons(beacons);
    });
    logPlayer.functionCalledWhenAccelerationUpdated([&](Acceleration acc){
        localizer->putAcceleration(acc);
    });
    logPlayer.functionCalledWhenAttitudeUpdated([&](Attitude att){
        localizer->putAttitude(att);
    });
    logPlayer.functionCalledWhenReset([&](Pose poseReset){
        localizer->resetStatus(poseReset);
    });
    logPlayer.run();
    
    if(opt.outputFilePath!=""){
        std::ofstream ofs(opt.outputFilePath);
        ofs << "timestamp," << Pose::header() << std::endl;
        ofs << userData.ss.str();
    }
    
    std::cout<<"Log play end"<<std::endl;
    return 0;
}