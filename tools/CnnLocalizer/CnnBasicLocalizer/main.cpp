//
//  main.cpp
//  CnnBasicLocalizer
//
//  Created by 石原辰也 on 2018/03/01.
//  Copyright © 2018 com.ibm.research.tokyo.ar. All rights reserved.
//

#include <iostream>
#include "CnnManager.hpp"
#include "BasicLocalizer.hpp"
#include "CnnFileUtil.hpp"
#include "CnnPoseUtil.hpp"
#include "MathUtils.hpp"
#include "DataUtils.hpp"
#include "LogUtil.hpp"
#include <getopt.h>
#include <boost/program_options.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace loc;

// These dimensions need to match those the model was trained with.
const int wanted_input_width = 224;
const int wanted_input_height = 224;
const int wanted_input_channels = 3;

const float input_mean = 128.0f;
const float input_std = 1.0f;
const float input_beacon_mean = 0.0f;
const float input_beacon_std = 1.0f;

typedef struct {
    std::string mapPath = "";
    std::string cnnSettingPath = "";
    std::string testPath = "";
    std::string outputPath = "";
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
    std::string restartLogPath = "";
    
    std::string localizerJSONPath = "";
    std::string outputLocalizerJSONPath ="";
    double magneticDeclination = NAN;
    bool verbose = false;
    BasicLocalizerOptions basicLocalizerOptions;

    ImageLocalizeMode imageLocalizeMode = IMAGE_BEACON;
    bool useImageMobilenet = false;
    bool useImageLstm = false;
} Option;

void printHelp() {
    std::cout << "Options for Basic Localizer" << std::endl;
    std::cout << " -h                  show this help" << std::endl;
    std::cout << " -m mapfile          set map data file" << std::endl;
    std::cout << " -c cnnfile          set CNN setting file" << std::endl;
    std::cout << " --train             force training parameters" << std::endl;
    std::cout << " --gptype <string>   set gptype [normal,light] for training" << std::endl;
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
    std::cout << " --restart=<outputpath>  use restart in log (outputpath is optional argument)" << std::endl;
    std::cout << " --lj                set localizer config json" << std::endl;
    std::cout << " --oj                output path for localizer config json" << std::endl;
    std::cout << " --declination       set magnetic declination to compute true north (east-positive, west-negative)" << std::endl;
    std::cout << " -v                  set verbosity" << std::endl;
    std::cout << " --finalize          finalize map data file" << std::endl;
    std::cout << " --mobilenet          use MobileNet" << std::endl;
    std::cout << " --lstm              use LSTM" << std::endl;
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
        {"restart",    optional_argument , NULL, 0},
        {"lj",         required_argument , NULL, 0},
        {"oj",         required_argument , NULL, 0},
        {"train",    no_argument , NULL, 0},
        {"declination",         required_argument , NULL, 0},
        //{"stdY",            required_argument, NULL,  0 },
        {"gptype",   required_argument , NULL, 0},
        {"finalize",   required_argument , NULL, 0},
        {"mobilenet",      no_argument, NULL, 0},
        {"lstm",      no_argument, NULL, 0},
        {0,         0,                 0,  0 }
    };
    
    while ((c = getopt_long(argc, argv, "m:c:t:ns:ho:rfv", long_options, &option_index )) != -1)
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
            if (strcmp(long_options[option_index].name, "declination") == 0){
                opt.magneticDeclination = atof(optarg);
            }
            if (strcmp(long_options[option_index].name, "finalize") == 0){
                opt.finalizeMapdata = true;
            }
            if (strcmp(long_options[option_index].name, "mobilenet") == 0){
                opt.useImageMobilenet = true;
            }
            if (strcmp(long_options[option_index].name, "lstm") == 0){
                opt.useImageLstm = true;
            }
            break;
        case 'h':
            printHelp();
            abort();
        case 'm':
            opt.mapPath.assign(optarg);
            break;
        case 'c':
            opt.cnnSettingPath.assign(optarg);
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
    
    std::map<int, std::string> imageCnnModelPathDict;
    std::map<int, std::string> imageBeaconSettingPathDict;
    std::map<std::pair<int,int>,int> beaconid_index_map;
    std::vector<float> beacon_rssis;
    std::shared_ptr<loc::CnnManager> cnnManager;
} MyData;

void functionCalledWhenUpdated(void *userData, loc::Status *pStatus){
    MyData *ud = (MyData*)userData;
    if (ud->opt->findRssiBias) {
        ud->status_list.insert(ud->status_list.end(), pStatus);
    } else {
        bool wasFloorUpdated = pStatus->wasFloorUpdated();
        auto refPose = pStatus->meanPose();
        int currentFloor = std::round(refPose->floor());
        auto recentPose = ud->recentPose;
        int recentFloor = std::round(recentPose.floor());
        if (wasFloorUpdated && !isnan(currentFloor) && recentFloor!=currentFloor) {
            if (ud->imageCnnModelPathDict.find(currentFloor)!=ud->imageCnnModelPathDict.end()
                && ud->imageBeaconSettingPathDict.find(currentFloor)!=ud->imageBeaconSettingPathDict.end()) {
                std::string imageCnnModelPath = ud->imageCnnModelPathDict[currentFloor];
                std::string imageBeaconSettingPath = ud->imageBeaconSettingPathDict[currentFloor];
                std::cout << "Load CNN model from " << imageCnnModelPath << std::endl;
                std::cout << "Load image beacon setting from " << imageBeaconSettingPath << std::endl;
                ud->cnnManager->init(imageCnnModelPath, ud->opt->imageLocalizeMode);
                
                ud->beaconid_index_map = loc::CnnFileUtil::parseBeaconSettingFile(imageBeaconSettingPath);
                ud->beacon_rssis.resize(ud->beaconid_index_map.size(), 0);
            } else {
                std::cout << "Cannot find CNN model, image beacon setting for the floor " << currentFloor << std::endl;
                ud->cnnManager->close();
                
                ud->beaconid_index_map.clear();
                ud->beacon_rssis.clear();
            }
        }
        
        auto locStatusStr = Status::locationStatusToString(pStatus->locationStatus());
        auto stepString = Status::stepToString(pStatus->step());
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
                *ud->out << "timestamp," << Pose::header() << ",lat,lng,status,step" << std::endl;
            }
            *ud->out << ts << "," << meanPoseGlobal << "," << locStatusStr << "," << stepString << std::endl;
            ud->writeCount = 1;
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
    ud.cnnManager = std::shared_ptr<loc::CnnManager>(new loc::CnnManager());
    
    auto resetBasicLocalizer = [](Option& opt, MyData& ud){
        std::shared_ptr<BasicLocalizer> localizer;
        
        std::ifstream ifs;
        if(opt.localizerJSONPath!=""){
            ifs = std::ifstream(opt.localizerJSONPath);
        }
        if(ifs.is_open()){
            std::cout << "localizerJSON=" << opt.localizerJSONPath << " is opened." << std::endl;
            BasicLocalizerParameters localizerParams;
            cereal::JSONInputArchive iarchive(ifs);
            iarchive(localizerParams);
            localizer = std::shared_ptr<BasicLocalizer>(new BasicLocalizer(localizerParams));
        }else{
            localizer = std::shared_ptr<BasicLocalizer>(new BasicLocalizer());
            localizer->localizeMode = opt.localizeMode;
            localizer->nSmooth = opt.nSmooth;
            localizer->smoothType = opt.smoothType;
            localizer->nStates = opt.nStates;
            // Some parameters must be set before calling setModel function.
            localizer->walkDetectSigmaThreshold = opt.walkDetectSigmaThreshold;
            
            localizer->meanRssiBias(opt.meanRssiBias);
            localizer->minRssiBias(opt.minRssiBias);
            localizer->maxRssiBias(opt.maxRssiBias);
            
            localizer->headingConfidenceForOrientationInit(0.5);
        }
        
        localizer->isVerboseLocalizer = opt.verbose;
        localizer->updateHandler(functionCalledWhenUpdated, &ud);
        localizer->forceTraining = opt.forceTraining;
        localizer->basicLocalizerOptions = opt.basicLocalizerOptions;
        localizer->finalizeMapdata = opt.finalizeMapdata;
        localizer->setModel(opt.mapPath, "./");
        localizer->normalFunction(opt.normFunc, opt.tDistNu); // set after calling setModel
        ud.latLngConverter = localizer->latLngConverter();
        
        if(opt.outputLocalizerJSONPath!=""){
            std::string strPath = opt.outputLocalizerJSONPath;
            std::ofstream ofs(strPath);
            if(ofs.is_open()){
                cereal::JSONOutputArchive oarchive(ofs);
                BasicLocalizerParameters localizerParams(*localizer);
                oarchive(localizerParams);
            }
        }

        std::size_t cnnSettingPathParentPos = opt.cnnSettingPath.find_last_of("/");
        std::string cnnSettingPathParent = "";
        if (cnnSettingPathParentPos!=std::string::npos) {
            cnnSettingPathParent = opt.cnnSettingPath.substr(0, cnnSettingPathParentPos);
        }
        CnnFileUtil::parseCnnSettingJsonFile(opt.cnnSettingPath, cnnSettingPathParent, opt.imageLocalizeMode,
                                             ud.imageCnnModelPathDict, ud.imageBeaconSettingPathDict);
        
        return localizer;
    };
    
    std::shared_ptr<BasicLocalizer> localizer = resetBasicLocalizer(opt, ud);
    // Parameter for reset log play
    double dx_reset = 1.0;
    double dy_reset = 1.0;
    double std_ori_reset = 10;
    
    if (opt.findRssiBias) {
        double step = (opt.maxRssiBias - opt.minRssiBias)/20;
        for(double rssiBias = opt.minRssiBias; rssiBias < opt.maxRssiBias; rssiBias += step) {
            localizer->meanRssiBias(rssiBias);
            localizer->minRssiBias(rssiBias-step);
            localizer->maxRssiBias(rssiBias+step);
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
                            if (iter->compare(0, 6, "Beacon") == 0) {
                                Beacons beacons = DataUtils::parseBeaconsCSV(*iter);
                                localizer->putBeacons(beacons);
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
        
    } else {
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
            } restarter;
            
            Beacons beaconsRecent;
            std::vector<double> errorsAtMarkers;

            assert(opt.testPath.length()>4 && boost::algorithm::ends_with(opt.testPath, ".log"));
            std::string testImageDir = opt.testPath.substr(0, opt.testPath.length()-4);
            
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
                            //std::cout << "LogReplay:" << beacons.timestamp() << ",Beacon," << beacons.size() << std::endl;
                            for(auto& b: beacons){
                                b.rssi( b.rssi() < 0 ? b.rssi() : -100);
                            }
                            localizer->putBeacons(beacons);
                            beaconsRecent = beacons;
                            // Compute likelihood at recent pose
                            {
                                auto recentPose = ud.recentPose;
                                auto obsModel = localizer->observationModel();
                                State sTmp(recentPose);
                                auto logLikelihood = obsModel->computeLogLikelihood(sTmp, beaconsRecent);
                            }
                            
                        }
                        // Parsing image values
                        if (logString.compare(0, 5, "Image") == 0 && beaconsRecent.size()>0) {
                            {
                                ud.beacon_rssis.resize(ud.beaconid_index_map.size(), 0);
                                for (int i=0; i<beaconsRecent.size(); i++) {
                                    loc::Beacon beacon = beaconsRecent[i];
                                    long rssi = -100;
                                    if (beacon.rssi() < 0) {
                                        rssi = beacon.rssi();
                                    }
                                    int major = beacon.major();
                                    int minor = beacon.minor();
                                    if (ud.beaconid_index_map.find(std::make_pair(major,minor))!=ud.beaconid_index_map.end()) {
                                        int beaconIndex = ud.beaconid_index_map[std::make_pair(major,minor)];
                                        float norm_rssi = rssi + 100;
                                        ud.beacon_rssis[beaconIndex] = (norm_rssi-input_beacon_mean)/input_beacon_std;
                                    }
                                }
                            }
                            
                            static const int resizeInputWidth = 455;
                            static const int resizeInputHeight = 256;
                            static const int cropInputSize = 224;
                            
                            long imageTimestamp = LogUtil::toImage(logString);
                            std::stringstream sstream;
                            sstream << testImageDir << "/" << imageTimestamp << ".jpg";
                            std::string imageFilePath = sstream.str();
                            cv::Mat image = cv::imread(imageFilePath);
                            assert(!image.empty());
                            
                            cv::Mat resizeImage;
                            cv::resize(image, resizeImage, cv::Size(resizeInputWidth, resizeInputHeight));
                            
                            int widthOffset = (resizeInputWidth - cropInputSize) / 2;
                            int heightOffset = (resizeInputHeight - cropInputSize) / 2;
                            cv::Mat cropImage = resizeImage(cv::Rect(widthOffset, heightOffset, cropInputSize, cropInputSize)).clone();
                            
                            {
                                const int image_channels = 4;
                                
                                assert(image_channels >= wanted_input_channels);
                                tensorflow::Tensor image_tensor;
                                float *out;
                                if (!opt.useImageLstm) {
                                    image_tensor = tensorflow::Tensor(tensorflow::DT_FLOAT,
                                                                      tensorflow::TensorShape({1, wanted_input_height, wanted_input_width, wanted_input_channels}));
                                    auto image_tensor_mapped = image_tensor.tensor<float, 4>();
                                    out = image_tensor_mapped.data();
                                } else {
                                    image_tensor = tensorflow::Tensor(tensorflow::DT_FLOAT,
                                                                    tensorflow::TensorShape({1, 1, wanted_input_height, wanted_input_width, wanted_input_channels}));
                                    auto image_tensor_mapped = image_tensor.tensor<float, 5>();
                                    out = image_tensor_mapped.data();
                                }
                                for (int y = 0; y < wanted_input_height; ++y) {
                                    float *out_row = out + (y * wanted_input_width * wanted_input_channels);
                                    for (int x = 0; x < wanted_input_width; ++x) {
                                        cv::Vec3b in_pixel = cropImage.at<cv::Vec3b>(y, x);
                                        float *out_pixel = out_row + (x * wanted_input_channels);
                                        for (int c = 0; c < wanted_input_channels; ++c) {
                                            out_pixel[c] = (in_pixel[c] - input_mean) / input_std;
                                        }
                                    }
                                }
                                
                                tensorflow::Tensor beacon_tensor;
                                float *beacon_out;
                                if (!opt.useImageLstm) {
                                    beacon_tensor = tensorflow::Tensor(tensorflow::DT_FLOAT,
                                                                       tensorflow::TensorShape({1, static_cast<long long>(ud.beaconid_index_map.size()), 1, 1}));
                                    auto beacon_tensor_mapped = beacon_tensor.tensor<float, 4>();
                                    beacon_out = beacon_tensor_mapped.data();
                                } else {
                                    beacon_tensor = tensorflow::Tensor(tensorflow::DT_FLOAT,
                                                                     tensorflow::TensorShape({1, 1, static_cast<long long>(ud.beaconid_index_map.size()), 1, 1}));
                                    auto beacon_tensor_mapped = beacon_tensor.tensor<float, 5>();
                                    beacon_out = beacon_tensor_mapped.data();
                                }
                                for (int i = 0; i < ud.beaconid_index_map.size(); i++) {
                                    beacon_out[i] = ud.beacon_rssis[i];
                                }
                                
                                auto startCnn = std::chrono::system_clock::now();
                                std::vector<double> result = ud.cnnManager->runCnn(image_tensor, beacon_tensor, opt.useImageMobilenet, opt.useImageLstm);
                                auto timeCnn = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-startCnn).count();
                                std::cerr << "Time to run CNN: " << timeCnn << "ms" << std::endl;
                                if (result.size()>0) {
                                    std::cerr << "CNN Estimation : " << result[0] << "," << result[1] << "," << result[2] << std::endl;
                                    
                                    auto status = localizer->getStatus();
                                    auto meanPose = status->meanPose();
                                    Pose estimatePose(*meanPose);
                                    estimatePose.x(result[0]).y(result[1]).z(result[2]);
                                    Eigen::Quaternion<double> estimateQ(result[3], result[4], result[5], result[6]);
                                    double estimateYaw = CnnPoseUtil::convertQuaternion2Yaw(estimateQ);
                                    estimatePose.orientation(estimateYaw);
                                    
                                    auto startPutImage = std::chrono::system_clock::now();
                                    localizer->putImageLocalizedPose(imageTimestamp, estimatePose);
                                    auto timePutImage = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-startPutImage).count();
                                    std::cerr << "Time to update PF by image : " << timePutImage << "ms" << std::endl;
                                } else {
                                    std::cerr << "CNN Estimation failed." << std::endl;
                                }
                            }
                        }
                        // Parsing acceleration values
                        if (logString.compare(0, 3, "Acc") == 0) {
                            Acceleration acc = LogUtil::toAcceleration(logString);
                            localizer->putAcceleration(acc);
                        }
                        // Parsing motion values
                        if (logString.compare(0, 6, "Motion") == 0) {
                            Attitude att = LogUtil::toAttitude(logString);
                            localizer->putAttitude(att);
                        }
                        
                        if (logString.compare(0, 9,"Altimeter") == 0){
                            Altimeter alt = LogUtil::toAltimeter(logString);
                            localizer->putAltimeter(alt);
                            //std::cout << "LogReplay:" << alt.timestamp() << ", Altimeter, " << alt.relativeAltitude() << "," << alt.pressure() << std::endl;
                        }
                        if (logString.compare(0, 7,"Heading") == 0){
                            Heading heading = LogUtil::toHeading(logString);
                            if(heading.trueHeading() < 0){ // (trueHeading==-1 is invalid.)
                                if(!isnan(opt.magneticDeclination)){
                                    double trueHeading = heading.magneticHeading() + opt.magneticDeclination;
                                    heading.trueHeading(trueHeading);
                                }else{
                                    std::stringstream ss;
                                    ss << "True heading (trueHeading=" << heading.trueHeading() << ") is invalid. Input declination argument";
                                    BOOST_THROW_EXCEPTION(LocException(ss.str()));
                                }
                            }
                            localizer->putHeading(heading);
                            LocalHeading localHeading = ud.latLngConverter->headingGlobalToLocal(heading);
                            //std::cout << "LogReplay:" << heading.timestamp() << ", Heading, " << heading.magneticHeading() << "," << heading.trueHeading() << "," << heading.headingAccuracy() << "(localHeading=" << localHeading.orientation() << ")" << std::endl;
                        }
                        if (logString.compare(0, 19, "DisableAcceleration") == 0){
                            std::vector<std::string> values;
                            boost::split(values, logString, boost::is_any_of(","));
                            int da = stoi(values.at(1));
                            long timestamp = stol(values.back());
                            if(da==1){
                                localizer->disableAcceleration(true,timestamp);
                            }else{
                                localizer->disableAcceleration(false,timestamp);
                            }
                            std::cout << "LogReplay:" << logString << std::endl;
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
                            
                            loc::Pose stdevPose;
                            stdevPose.x(dx_reset).y(dy_reset).orientation(std_ori_reset/180*M_PI);
                            
                            localizer->resetStatus(newPose, stdevPose);
                        }
                        if (opt.usesRestart && logString.compare(0, 7, "Restart") == 0){
                            // "Restart",timestamp
                            std::vector<std::string> values;
                            boost::split(values, logString, boost::is_any_of(","));
                            long timestamp = stol(values.at(1));
                            std::cout << "LogReplay: " << timestamp << ",Restart," << std::endl;
                            localizer = resetBasicLocalizer(opt, ud);
                            restarter.reset();
                            restarter.counter=1;
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
                                auto obsModel = localizer->observationModel();
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
                        if (logString.compare(0, 4, "Note") == 0){
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

