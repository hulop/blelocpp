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
    std::string workingDir = ".";
    double meanRssiBias = 0.0;
    double minRssiBias = -10;
    double maxRssiBias = 10;
    NormalFunction normFunc = NORMAL;
    double tDistNu = 3;
    int nSmooth = 10;
    int nStates = 1000;
    SmoothType smoothType = SMOOTH_LOCATION;
    bool findRssiBias = false;
    LocalizeMode localizeMode = ONESHOT;
    double walkDetectSigmaThreshold = 0.6;
    bool usesReset = false;
    bool usesRestart = false;
    bool forceTraining = false;
    bool finalizeMapdata = false;
    bool binaryOutput = false;
    std::string trainedFile = "";
    std::string finalizedFile = "";
    std::string binaryFile = "";
    std::set<std::string> binarizeTargets{"model"};
    std::string restartLogPath = "";
    bool longLog = false;
    
    std::string localizerJSONPath = "";
    std::string outputLocalizerJSONPath ="";
    double magneticDeclination = NAN;
    bool verbose = false;
    BasicLocalizerOptions basicLocalizerOptions;
    int skipBeacon = 0;
    long maxBeaconInterval = std::numeric_limits<long>::max();
    int runMode = 0;
    std::string outDir = "";
} Option;

void printHelp() {
    std::cout << "Options for Basic Localizer" << std::endl;
    std::cout << " -h                  show this help" << std::endl;
    std::cout << " -m mapfile          set map data file" << std::endl;
    std::cout << " --wd <dir>          working directory (default .)" << std::endl;
    std::cout << " --train [<name>]    force training parameters (save to <name>)" << std::endl;
    std::cout << " --gptype <string>   set gptype [normal,light] for training" << std::endl;
    std::cout << " --knltype <string>  set knltype [all,clustered] for training" << std::endl;
    std::cout << " --overlapScale <double> set the overlap scale for gplight (default=0.001)" << std::endl;
    std::cout << " --noPathLoss        train model without pathloss model" << std::endl;
    std::cout << " --mattype <string>  set matrix type [dense,sparse] for observation model" << std::endl;
    std::cout << " -t testfile         set test csv data file" << std::endl;
    std::cout << " --maxInterval       set max timestamp interval between beacon inputs [ms] (default=inf)" << std::endl;
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
    std::cout << " --restart=<outputpath>  use restart in log (outputpath is optional argument)" << std::endl;
    std::cout << " --lj                set localizer config json" << std::endl;
    std::cout << " --oj                output path for localizer config json" << std::endl;
    std::cout << " --declination       set magnetic declination to compute true north (east-positive, west-negative)" << std::endl;
    std::cout << " -v                  set verbosity" << std::endl;
    std::cout << " --finalize [<name>] finalize map data file (save to <name>)" << std::endl;
    std::cout << " --binary [<name>]   save model as binary" << std::endl;
    std::cout << " --btarget [name ...] set binarize targets " << std::endl;
    std::cout << " --skip              set skip count of initial beacon inputs" << std::endl;
}

Option parseArguments(int argc, char *argv[]){
    Option opt;
    
    if (strcmp(argv[1], "predict")==0){
        opt.runMode = 1;
    }else{
        opt.runMode = 0;
    }
    
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
        {"restart",    optional_argument , NULL, 0},
        {"lj",         required_argument , NULL, 0},
        {"oj",         required_argument , NULL, 0},
        {"train",    optional_argument , NULL, 0},
        {"declination",         required_argument , NULL, 0},
        //{"stdY",            required_argument, NULL,  0 },
        {"gptype",   required_argument , NULL, 0},
        {"knltype",   required_argument , NULL, 0},
        {"overlapScale", required_argument , NULL, 0},
        {"mattype",   required_argument , NULL, 0},
        {"noPathLoss",   no_argument , NULL, 0},
        {"maxInterval",   required_argument , NULL, 0},
        {"finalize",   optional_argument , NULL, 0},
        {"binary",   optional_argument , NULL, 0},
        {"btarget",   required_argument , NULL, 0},
        {"wd",   required_argument , NULL, 0},
        {"skip",         required_argument , NULL, 0},
        {"vl",         required_argument , NULL, 0},
        {"outDir",     required_argument , NULL, 0},
        {0,         0,                 0,  0 }
    };

    while ((c = getopt_long(argc, argv, "m:t:ns:ho:rfv", long_options, &option_index )) != -1)
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
            if (strcmp(long_options[option_index].name, "restart") == 0){
                opt.usesRestart = true;
                if(optarg!=NULL){
                    opt.restartLogPath.assign(optarg);
                }else{
                    std::cout << "restart log path is null." << std::endl;
                }
            }
            if (strcmp(long_options[option_index].name, "lj") == 0){
                opt.localizerJSONPath.assign(optarg);
            }
            if (strcmp(long_options[option_index].name, "oj") == 0){
                opt.outputLocalizerJSONPath.assign(optarg);
            }
            if (strcmp(long_options[option_index].name, "train") == 0){
                opt.forceTraining = true;
                if (optarg) {
                    opt.trainedFile.assign(optarg);
                }
            }
            if (strcmp(long_options[option_index].name, "gptype") == 0){
                std::string str(optarg);
                if(str=="normal"){
                    opt.basicLocalizerOptions.gpType = GPNORMAL;
                }else if(str=="light"){
                    opt.basicLocalizerOptions.gpType = GPLIGHT;
                }else{
                    std::cerr << "Unknown gptype: " << optarg << std::endl;
                    abort();
                }
            }
            if (strcmp(long_options[option_index].name, "knltype") == 0){
                std::string str(optarg);
                if(str=="all"){
                    opt.basicLocalizerOptions.knlType = KNLALL;
                }else if(str=="clustered"){
                    opt.basicLocalizerOptions.knlType = KNLCLUSTERED;
                }else{
                    std::cerr << "Unknown knltype: " << optarg << std::endl;
                    abort();
                }
            }
            if (strcmp(long_options[option_index].name, "overlapScale") == 0){
                opt.basicLocalizerOptions.overlapScale = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "noPathLoss") == 0){
                opt.basicLocalizerOptions.noPathLoss = true;
            }
            if (strcmp(long_options[option_index].name, "mattype") == 0){
                std::string str(optarg);
                if(str=="dense"){
                    opt.basicLocalizerOptions.matType = DENSE;
                }else if(str=="sparse"){
                    opt.basicLocalizerOptions.matType = SPARSE;
                }else{
                    std::cerr << "Unknown mattype: " << optarg << std::endl;
                    abort();
                }
            }
            if (strcmp(long_options[option_index].name, "declination") == 0){
                opt.magneticDeclination = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "finalize") == 0){
                opt.finalizeMapdata = true;
                if (optarg) {
                    opt.finalizedFile.assign(optarg);
                }
            }
            if (strcmp(long_options[option_index].name, "binary") == 0){
                opt.binaryOutput = true;
                if (optarg) {
                    opt.binaryFile.assign(optarg);
                }
            }
            if (strcmp(long_options[option_index].name, "btarget") == 0){
                std::string btarget(optarg);
                std::stringstream ss(btarget);
                std::string field;
                std::set<std::string> candidates{"model","building"};
                opt.binarizeTargets.clear();
                int err = 0;
                while (std::getline(ss, field, ',')) {
                    if(candidates.count(field)){
                        opt.binarizeTargets.insert(field);
                    }else{
                        std::cerr << "unrecognized binarize target = " << field << std::endl;
                        err=1;
                    }
                }
                if(err!=0){
                    exit(1);
                }
            }
            if (strcmp(long_options[option_index].name, "wd") == 0){
                opt.workingDir.assign(optarg);
            }
            if (strcmp(long_options[option_index].name, "skip") == 0){
                opt.skipBeacon = atoi(optarg);
            }
            if (strcmp(long_options[option_index].name, "vl") == 0){
                opt.longLog = true;
            }
            if (strcmp(long_options[option_index].name, "outDir") == 0){
                opt.outDir.assign(optarg);
            }
            if (strcmp(long_options[option_index].name, "maxInterval") == 0){
                opt.maxBeaconInterval = atoi(optarg);
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
        case 'v':
            opt.verbose = true;
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
    std::function<void(Status&)> func;
    int writeCount = 0;
    Beacons recentBeacons;
} MyData;

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    MyData *ud = (MyData*)userData;
    if (ud->opt->findRssiBias) {
        ud->status_list.insert(ud->status_list.end(), pStatus);
    } else {
        auto locStatusStr = Status::locationStatusToString(pStatus->locationStatus());
        auto stepString = Status::stepToString(pStatus->step());
        double wscore = pStatus->wscore;
        double ascore = pStatus->ascore;
        
        // information about beacons
        auto bs = ud->recentBeacons;
        size_t nBeacons = bs.size();
        size_t nStrong = 0;
        for(const auto& b: bs){
            if(-100 < b.rssi() && b.rssi() < 0){
                nStrong++;
            }
        }
        std::stringstream ssBeacons;
        ssBeacons << nBeacons << "," << nStrong;
        for(const auto& b: bs){
            ssBeacons << "," << b.major() << "," << b.minor() << "," << b.rssi();
        }
        auto strBeacons = ssBeacons.str();
        
        //std::cout << "locationStatus=" << locStatusStr << std::endl;
        //if(pStatus->step()==Status::FILTERING_WITH_RESAMPLING ||
        //   pStatus->step()==Status::FILTERING_WITHOUT_RESAMPLING ||
        //    pStatus->step()==Status::RESET){
        //if(true){
        
        if(pStatus->step()!=Status::OTHER){
            auto ts = pStatus->timestamp();
            auto meanLocGlobal = ud->latLngConverter->localToGlobal(*pStatus->meanLocation());
            auto meanPoseGlobal = ud->latLngConverter->localToGlobal(*pStatus->meanPose());
            
            if(ud->writeCount==0){
                *ud->out << "timestamp," << Pose::header() << ",lat,lng,status,step";
                if(ud->opt->longLog){
                    *ud->out <<",wscore,ascore,status_num,nBeacons,nStrong";
                }
                *ud->out << std::endl;
                ud->writeCount = 1;
            }
            
            *ud->out << ts << "," << meanPoseGlobal << "," << locStatusStr << "," << stepString;
            if(ud->opt->longLog){
                *ud->out << "," << wscore << "," << ascore << "," << pStatus->locationStatus() << "," << strBeacons;
            }
            *ud->out << std::endl;
            
            ud->recentPose = *pStatus->meanPose();
            if(ud->func != NULL){
                ud->func(*pStatus);
                ud->func = NULL;
            }
        }
        
        for(const State& s: *pStatus->states()) {
            auto gs = ud->latLngConverter->localToGlobal(s);
            assert(gs.lat() != 0);
            assert(gs.lng() != 0);
        }
    }
}


class Restarter{
public:
    int counter=0;
    long tsStart;
    long tsEnd;
    std::shared_ptr<Location> markerLocStart;
    std::shared_ptr<Location> markerLocEnd;
    std::shared_ptr<Pose> repPoseStart;
    std::shared_ptr<Pose> repPoseEnd;
    States statesStart;
    States statesEnd;
    
    void reset(){
        markerLocStart.reset();
        markerLocEnd.reset();
        repPoseStart.reset();
        repPoseEnd.reset();
        counter=0;
    }
};


int findRssiBias(Option& opt, MyData& ud, BasicLocalizer& localizer){
    double step = (opt.maxRssiBias - opt.minRssiBias)/20;
    for(double rssiBias = opt.minRssiBias; rssiBias < opt.maxRssiBias; rssiBias += step) {
        localizer.meanRssiBias(rssiBias);
        localizer.minRssiBias(rssiBias-step);
        localizer.maxRssiBias(rssiBias+step);
        ud.status_list.empty();
        
        if(!opt.testPath.empty()){
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
                        if (iter->compare(0, 7, "Beacon,") == 0) {
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
        }else{
            std::cout << "test file is not specified" << std::endl;
        }
    }
    return 0;
}

BasicLocalizer resetBasicLocalizer(Option& opt, MyData& ud){
    BasicLocalizer localizer;
    
    std::ifstream ifs;
    if(opt.localizerJSONPath!=""){
        ifs = std::ifstream(opt.localizerJSONPath);
    }
    if(ifs.is_open()){
        std::cout << "localizerJSON=" << opt.localizerJSONPath << " is opened." << std::endl;
        BasicLocalizerParameters localizerParams;
        cereal::JSONInputArchive iarchive(ifs);
        iarchive(localizerParams);
        localizer = BasicLocalizer(localizerParams);
    }else{
        localizer.localizeMode = opt.localizeMode;
        localizer.nSmooth = opt.nSmooth;
        localizer.smoothType = opt.smoothType;
        localizer.nStates = opt.nStates;
        // Some parameters must be set before calling setModel function.
        localizer.walkDetectSigmaThreshold = opt.walkDetectSigmaThreshold;
        
        localizer.meanRssiBias(opt.meanRssiBias);
        localizer.minRssiBias(opt.minRssiBias);
        localizer.maxRssiBias(opt.maxRssiBias);
        
        localizer.headingConfidenceForOrientationInit(0.5);
    }
    
    localizer.isVerboseLocalizer = opt.verbose;
    localizer.updateHandler(functionCalledWhenUpdated, &ud);
    localizer.forceTraining = opt.forceTraining;
    localizer.basicLocalizerOptions = opt.basicLocalizerOptions;
    localizer.finalizeMapdata = opt.finalizeMapdata;
    localizer.binaryOutput = opt.binaryOutput;
    localizer.binaryFile = opt.binaryFile;
    localizer.trainedFile = opt.trainedFile;
    localizer.finalizedFile = opt.finalizedFile;
    localizer.binarizeTargets = opt.binarizeTargets;
    localizer.setModel(opt.mapPath, opt.workingDir);
    localizer.normalFunction(opt.normFunc, opt.tDistNu); // set after calling setModel
    ud.latLngConverter = localizer.latLngConverter();
    
    if(opt.outputLocalizerJSONPath!=""){
        std::string strPath = opt.outputLocalizerJSONPath;
        std::ofstream ofs(strPath);
        if(ofs.is_open()){
            cereal::JSONOutputArchive oarchive(ofs);
            BasicLocalizerParameters localizerParams(localizer);
            oarchive(localizerParams);
        }
    }
    
    return localizer;
}

int runPredict(Option& opt, MyData& ud, BasicLocalizer& localizer){
    if(!opt.testPath.empty()){
        std::ifstream ifs(opt.testPath);
        std::string str;
        if (ifs.fail())
        {
            std::cerr << "test file is unable to read: " << opt.testPath << std::endl;
            return -1;
        }
        
        auto obsModel = localizer.observationModel();
        auto beaconFilter = localizer.beaconFilter; // apply the same becon filter
        auto statusInitializer = localizer.statusInitializer;
        
        double radius2D = 30.0;
        double floorDiffMax = 100.0;
        
        int beaconReceiveCount = 0;
        while (getline(ifs, str))
        {
            try {
                std::vector<std::string> v;
                boost::split(v, str, boost::is_any_of(" "));
                if(v.size() > 3){
                    std::string logString = v.at(3);
                    // Parsing beacons values
                    if (logString.compare(0, 7, "Beacon,") == 0) {
                        Beacons beacons = LogUtil::toBeacons(logString);
                        for(auto& b: beacons){
                            b.rssi( b.rssi() < 0 ? b.rssi() : -100);
                        }
                        beaconReceiveCount++;
                        auto ts = beacons.timestamp();
                        
                        if(opt.skipBeacon!=0){
                            if(beaconReceiveCount % opt.skipBeacon != 0){
                                continue;
                            }
                        }
                        
                        auto beaconsTemp = beaconFilter->filter(beacons);
                        auto closeLocations = statusInitializer->extractLocationsCloseToBeacons(beaconsTemp, radius2D, floorDiffMax);
                        States evalStates;
                        for(const auto& loc: closeLocations){
                            evalStates.push_back(State(loc));
                        }
                        
                        auto logLLs = obsModel->computeLogLikelihood(evalStates, beaconsTemp);
                        
                        std::map<BeaconId, std::vector<double>> idTologLL;
                        for(const auto& b: beaconsTemp){
                            Beacons bs;
                            bs.timestamp(beaconsTemp.timestamp());
                            bs.push_back(b);
                            std::vector<double> logLL = obsModel->computeLogLikelihood(evalStates, bs);
                            idTologLL[b.id()] = logLL;
                        }
                        
                        std::cout << beacons.timestamp() << ",Beacons," << beaconsTemp.size() << std::endl;
                        if(!opt.outDir.empty()){
                            std::string outFilePath = opt.outDir + "/" + "logLikelihood_timestamp" + std::to_string(ts) + ".csv";
                            std::ofstream ofs(outFilePath);
                            if (ofs.fail()) {
                                std::cerr << "output file is unable to write: " << outFilePath << std::endl;
                                return -1;
                            }
                            
                            // header
                            // timestamp,x,y,floor,logLikelihood,n_beacons,beaconID0,beaconID1,...,beaconIDN
                            ofs << "timestamp,x,y,floor,logLikelihood,n_beacons";
                            for(const auto& b: beaconsTemp){
                                if(b.id().buuid()==boost::uuids::nil_uuid()){
                                    ofs << "," << b.id().major() << "-" << b.id().minor();;
                                }else{
                                    ofs << "," << b.id().toString();
                                }
                            }
                            ofs << std::endl;
                            
                            // data
                            // timestamp,x,y,floor,sum_logLikelihood,n_beacons,logLL_for_beaconID0,...,logLL_for_beaconIDN
                            for(int i = 0; i<evalStates.size() ; i++){
                                const auto& s = evalStates.at(i);
                                const auto& logLL = logLLs.at(i);
                                ofs << ts << "," << s.x() << "," << s.y() << "," << s.floor() << "," << logLL;
                                ofs << "," << beaconsTemp.size() ;
                                for(int j = 0; j < beaconsTemp.size() ; j++){
                                    ofs << "," << idTologLL[beaconsTemp.at(j).id()].at(i) ;
                                }
                                ofs << std::endl;
                            }
                            ofs.close();
                        }
                    }
                }
            }catch (LocException& e){
                std::cerr << boost::diagnostic_information(e) << std::endl;
            }catch (std::invalid_argument e){
                std::cerr << e.what() << std::endl;
                std::cerr << "error in parse log file" << std::endl;
            }
        }
    }else{
        std::cout << "test file is not specified" << std::endl;
    }
    return 0;
}

int main(int argc, char * argv[]) {
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
    size_t lastindex = opt.mapPath.find_last_of(".");
    std::string rawname = opt.mapPath.substr(0, lastindex);
    if (opt.binaryOutput && opt.binaryFile.length() == 0) {
        opt.binaryFile.assign(rawname+".BinaryModelData.bin");
    }
    if (opt.forceTraining && opt.trainedFile.length() == 0) {
        opt.trainedFile.assign(rawname+".trained.json");
    }
    if (opt.finalizeMapdata && opt.finalizedFile.length() == 0) {
        opt.finalizedFile.assign(rawname+".finalized.json");
    }

    BasicLocalizer localizer;
    try{
        localizer = resetBasicLocalizer(opt, ud);
    }catch(std::string& str){
        std::cerr << str << std::endl;
        return -1;
    }catch(const char* ch){
        std::cerr << ch << std::endl;
        return -1;
    }catch(LocException& e){
        std::cerr << boost::diagnostic_information(e) << std::endl;
        return -1;
    }
    
    if (opt.findRssiBias) {
        int ret = findRssiBias(opt, ud, localizer);
        if(ret!=0){
            return ret;
        }
    } else if (opt.runMode==1){
        int ret = runPredict(opt, ud, localizer);
        if(ret!=0){
            return ret;
        }
    } else {
        // Parameter for reset log play
        double dx_reset = 1.0;
        double dy_reset = 1.0;
        double std_ori_reset = 10;
        
        // Sequential Log Replay
        if(!opt.testPath.empty()){
            std::ifstream ifs(opt.testPath);
            std::string str;
            if (ifs.fail())
            {
                std::cerr << "test file is unable to read: " << opt.testPath << std::endl;
                return -1;
            }
            
            std::unique_ptr<std::ostream> restartLogOut;
            if (opt.restartLogPath.length()>0){
                restartLogOut.reset(new std::ofstream(opt.restartLogPath));
                if(restartLogOut->fail()){
                    std::cerr << "restart log output file is unable to write: " << opt.outputPath << std::endl;
                    return -1;
                }else{
                    *restartLogOut << "timestamp,marker_type,x_t,y_t,z_t,floor_t,ori_t,x_e,y_e,z_e,floor_e,orientation_e,...,orientation_e_stdev" << std::endl;
                }
            }
            
            Restarter restarter;
            Beacons beaconsRecent;
            std::vector<double> errorsAtMarkers;
            
            int beaconReceiveCount = 0;
            while (getline(ifs, str))
            {
                try {
                    std::vector<std::string> v;
                    boost::split(v, str, boost::is_any_of(" "));
                    if(v.size() > 3){
                        std::string logString = v.at(3);
                        // Parsing beacons values
                        if (logString.compare(0, 7, "Beacon,") == 0) {
                            Beacons beacons = LogUtil::toBeacons(logString);
                            //std::cout << "LogReplay:" << beacons.timestamp() << ",Beacon," << beacons.size() << std::endl;
                            for(auto& b: beacons){
                                b.rssi( b.rssi() < 0 ? b.rssi() : -100);
                            }
                            
                            // check timeout for beacon inputs and reset the localizer if required
                            auto diffTimestamp = beacons.timestamp() - beaconsRecent.timestamp();
                            if(0<beaconReceiveCount && diffTimestamp > opt.maxBeaconInterval){
                                if(opt.verbose){
                                    std::cout << "Localizer status was reset because diffTimestamp=" << diffTimestamp << " was larger than maxInterval=" << opt.maxBeaconInterval << "." <<  std::endl;
                                }
                                localizer.resetAllStatus();
                            }
                            
                            beaconReceiveCount++;
                            if(opt.skipBeacon<beaconReceiveCount){
                                ud.recentBeacons = beacons; // ud.recentBeacons is used in the callback function.
                                localizer.putBeacons(beacons);
                            }else{
                                std::cout << "First "<< beaconReceiveCount << " beacon input was skipped." << std::endl;
                            }
                            beaconsRecent = beacons; // beaconsRecent is used in this while loop.
                            
                            // Compute likelihood at recent pose
                            //{
                            //    auto recentPose = ud.recentPose;
                            //    auto obsModel = localizer.observationModel();
                            //    State sTmp(recentPose);
                            //    auto logLikelihood = obsModel->computeLogLikelihood(sTmp, beaconsRecent);
                            //}
                        }
                        // Parsing acceleration values
                        if (logString.compare(0, 4, "Acc,") == 0) {
                            Acceleration acc = LogUtil::toAcceleration(logString);
                            localizer.putAcceleration(acc);
                        }
                        // Parsing motion values
                        if (logString.compare(0, 7, "Motion,") == 0) {
                            Attitude att = LogUtil::toAttitude(logString);
                            localizer.putAttitude(att);
                        }
                        
                        if (logString.compare(0, 10,"Altimeter,") == 0){
                            Altimeter alt = LogUtil::toAltimeter(logString);
                            localizer.putAltimeter(alt);
                            //std::cout << "LogReplay:" << alt.timestamp() << ", Altimeter, " << alt.relativeAltitude() << "," << alt.pressure() << std::endl;
                        }
                        if (logString.compare(0, 8,"Heading,") == 0){
                            Heading heading = LogUtil::toHeading(logString);
                            
                            // do not update the trueHeading value in this block because heading.trueHeading() < 0 is checked later.
                            if(heading.trueHeading() < 0){ // (trueHeading==-1 is invalid.)
                                Anchor anchor = ud.latLngConverter->anchor();
                                if(std::isnan(anchor.magneticDeclination)){
                                    if(!isnan(opt.magneticDeclination)){
                                        // overwrite declination to the anchor if it is set as an argument value.
                                        anchor.magneticDeclination = opt.magneticDeclination;
                                        ud.latLngConverter->anchor(anchor);
                                    }else{
                                        // if declination is not set in the anchor and the argument value.
                                        std::stringstream ss;
                                        ss << "True heading (trueHeading=" << heading.trueHeading() << ") is invalid. Input declination argument";
                                        BOOST_THROW_EXCEPTION(LocException(ss.str())); // a valid declination value is required in the command line tool.
                                    }
                                }
                            }
                            localizer.putHeading(heading);
                            //LocalHeading localHeading = ud.latLngConverter->headingGlobalToLocal(heading);
                            //std::cout << "LogReplay:" << heading.timestamp() << ", Heading, " << heading.magneticHeading() << "," << heading.trueHeading() << "," << heading.headingAccuracy() << "(localHeading=" << localHeading.orientation() << ")" << std::endl;
                        }
                        if (logString.compare(0, 20, "DisableAcceleration,") == 0){
                            std::vector<std::string> values;
                            boost::split(values, logString, boost::is_any_of(","));
                            int da = stoi(values.at(1));
                            long timestamp = stol(values.back());
                            if(da==1){
                                localizer.disableAcceleration(true,timestamp);
                            }else{
                                localizer.disableAcceleration(false,timestamp);
                            }
                            std::cout << "LogReplay:" << logString << std::endl;
                        }
                        if (opt.usesReset && logString.compare(0, 6, "Reset,") == 0) {
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
                            
                            loc::Pose stdevPose;
                            stdevPose.x(dx_reset).y(dy_reset).orientation(std_ori_reset/180*M_PI);
                            
                            localizer.resetStatus(newPose, stdevPose);
                        }
                        if (opt.usesRestart && logString.compare(0, 8, "Restart,") == 0){
                            // "Restart",timestamp
                            std::vector<std::string> values;
                            boost::split(values, logString, boost::is_any_of(","));
                            long timestamp = stol(values.at(1));
                            std::cout << "LogReplay: " << timestamp << ",Restart," << std::endl;
                            localizer = resetBasicLocalizer(opt, ud);
                            restarter.reset();
                            restarter.counter=1;
                        }
                        if (logString.compare(0, 7, "Marker,") == 0){
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
                            
                            std::stringstream ss;
                            ss << "LogReplay:" << timestamp << ",Marker,";
                            ss << std::setprecision(10) << lat << "," << lng;
                            ss << "," << floor;
                            
                            if(restarter.counter==0){
                                auto recentPose = ud.recentPose;
                                double d2D = Location::distance2D(markerLoc, recentPose);
                                double dFloor = Location::floorDifference(markerLoc, recentPose);
                                errorsAtMarkers.push_back(d2D);
                                std::cout << ",d2D=" << d2D << ",dFloor=" <<dFloor ;
                            }else if(restarter.counter==1){
                                restarter.markerLocStart.reset(new Location(markerLoc));
                                //ud.func = [&](long ts, Pose pose, States states){
                                ud.func = [&](Status& status){
                                    restarter.tsStart = status.timestamp();
                                    restarter.repPoseStart.reset(new Pose(*status.meanPose()));
                                    restarter.statesStart = *status.states();
                                };
                                restarter.counter=2;
                            }else if(restarter.counter==2){
                                if(restarter.markerLocStart && !restarter.repPoseStart){
                                    ss << "Marker duplication was found. Marker=" << markerLoc << " was not used.";
                                }else{
                                    restarter.markerLocEnd.reset(new Location(markerLoc));
                                    //ud.func = [&](long ts, Pose pose, States states){
                                    ud.func = [&](Status& status){
                                        restarter.tsEnd = status.timestamp();
                                        restarter.repPoseEnd.reset(new Pose(*status.meanPose()));
                                        restarter.statesEnd = *status.states();
                                        restarter.counter=3;
                                    };
                                }
                            }
                            std::cout << ss.str() << std::endl;
                            
                            // Compare prediction at marker location with recent beacons
                            {
                                std::cout << std::endl;
                                auto obsModel = localizer.observationModel();
                                State sTmp(markerLoc);
                                auto predictions = obsModel->predict(sTmp, beaconsRecent);
                                std::cout << "(minor,meas,pred)=";
                                for(auto& b : beaconsRecent){
                                    if(b.rssi() > -100){
                                        auto pred = predictions.at(b.id());
                                        std::cout << "(" << b.minor() << "," << b.rssi()  << "," << pred.mean() << "),";
                                    }
                                }
                                std::cout << std::endl;
                            }
                        }
                        if (logString.compare(0, 5, "Note,") == 0){
                            // "Note", note_string
                            std::stringstream ss;
                            ss << "LogReplay: " << logString;
                            if(1<=restarter.counter){
                                // When "Note" is detected after restarting, the error must not be evaluated at the step.
                                ss << " Note is detected after restart. Markers before the Note will not be evaluated.";
                                restarter.reset();
                            }
                            std::cout << ss.str() << std::endl;
                        }
                        
                        if(restarter.counter==3){
                            if(restarter.repPoseStart && restarter.repPoseEnd){
                                std::cout << "LogReplay: Evaluate restart: "
                                << "marker0=" << *restarter.markerLocStart << ","
                                << "marker1=" << *restarter.markerLocEnd << ","
                                << "repPoseStart=" << *restarter.repPoseStart << ","
                                << "repPoseEnd=" << *restarter.repPoseEnd << std::endl;
                                
                                Pose markerPose0(*restarter.markerLocStart);
                                Pose markerPose1(*restarter.markerLocEnd);
                                double dy = restarter.markerLocEnd->y() - restarter.markerLocStart->y();
                                double dx = restarter.markerLocEnd->x() - restarter.markerLocStart->x();
                                double oriPath = std::atan2(dy, dx);
                                markerPose0.orientation(oriPath);
                                markerPose1.orientation(oriPath);
                                
                                double d2D = Location::distance2D(*restarter.markerLocStart, *restarter.repPoseStart);
                                double dFloor = Location::floorDifference(*restarter.markerLocStart, *restarter.repPoseStart);
                                std::cout << "  Initial location: dist2D=" << d2D <<  ", floorDiff=" << dFloor << std::endl;
                                
                                d2D = Location::distance2D(*restarter.markerLocEnd, *restarter.repPoseEnd);
                                dFloor = Location::floorDifference(*restarter.markerLocEnd, *restarter.repPoseEnd);
                                std::cout << "  Reached location: dist2D=" << d2D <<  ", floorDiff=" << dFloor << std::endl;
                                
                                double oriRep = restarter.repPoseEnd->orientation();
                                std::cout << "  Orientation: theta_true=" << oriPath << ", theta_est=" << oriRep << std::endl;
                                
                                auto wnParamStart = Pose::computeWrappedNormalParameter(restarter.statesStart);
                                double oriStdStart = wnParamStart.stdev();
                                auto wnParamEnd = Pose::computeWrappedNormalParameter(restarter.statesEnd);
                                double oriStdEnd = wnParamEnd.stdev();
                                
                                if(restartLogOut){
                                    *restartLogOut << restarter.tsStart << ",Restart," << *restarter.markerLocStart
                                    << "," << oriPath
                                    << "," << *restarter.repPoseStart
                                    << "," << oriStdStart <<  std::endl;
                                    *restartLogOut << restarter.tsEnd << ",Stop," << *restarter.markerLocEnd
                                    << "," << oriPath
                                    << "," << *restarter.repPoseEnd
                                    << "," << oriStdEnd << std::endl;
                                }
                                
                                restarter.reset();
                            }
                        }
                    }
                }catch (LocException& e){
                    std::cerr << boost::diagnostic_information(e) << std::endl;
                }catch (std::invalid_argument e){
                    std::cerr << e.what() << std::endl;
                    std::cerr << "error in parse log file" << std::endl;
                }
            }
        }else{
            std::cout << "test file is not specified" << std::endl;
        }
    }
    return 0;
}
