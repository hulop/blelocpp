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
#include <fstream>
#include <getopt.h>
#include "picojson.h"
#include "DataUtils.hpp"
#include "LatLngConverter.hpp"

using namespace loc;

struct Option{
    std::string from;
    std::string to;
    std::string samples_path;
    std::string beacons_path;
    
    std::string samples_out_path;
    std::string beacons_out_path;
};

std::string lastComponent(char *cstr) {
    std::string str(cstr);
    std::size_t found = str.find_last_of("/");
    return str.substr(found+1);
}

void printHelp(std::string command){
    std::cout << "Convert coordinate of samples or beacons between two anchors." << std::endl;
    std::cout << " -h                   show this help" << std::endl;
    std::cout << " -f anchorFile        set anchor file used to convert coordinate from local to global coordinate" << std::endl;
    std::cout << " -t anchorFile        set anchor file used to convert coordinate from global to local coordinate" << std::endl;
    std::cout << " -s samplesFile       set samples file to be converted" << std::endl;
    std::cout << " -b beaconsFile       set beacons file to be converted" << std::endl;
    std::cout << " --so samplesOutputFile" << std::endl;
    std::cout << " --bo beaconsOutputFile" << std::endl;
    std::cout << std::endl;
    std::cout << "Example" << std::endl;
    std::cout << "$ " << command << " -f anchor_from.json -t anchor_to.json -s samples.csv -b beacons.csv" << std::endl;
    std::cout << std::endl;
}

Anchor parseJsonAnchor(picojson::object& jsonObject){
    Anchor anchor;
    double lat = jsonObject["latitude"].get<double>();
    double lng = jsonObject["longitude"].get<double>();
    double rotate = jsonObject["rotate"].get<double>();
    anchor.latlng.lat = lat;
    anchor.latlng.lng = lng;
    anchor.rotate = rotate;
    return anchor;
}

Anchor parseJsonAnchor(std::string& filepath){
    std::string str = DataUtils::fileToString(filepath);
    picojson::value value;
    std::string err = picojson::parse(value, str);
    if(! err.empty()){
        std::cout << "error" << std::endl;
    }
    if(! value.is<picojson::object>()){
        std::cout << "invalid json" << std::endl;
    }
    picojson::object& jobj = value.get<picojson::object>();
    picojson::object anchorObj = jobj["anchor"].get<picojson::object>();
    Anchor anchor = parseJsonAnchor(anchorObj);
    return anchor;
}

#define SAMPLES_OUT "so"
#define BEACONS_OUT "bo"

Option parseArguments(int argc, char *argv[]){
    Option opt;
    int c = 0;
    int option_index = 0;
    struct option long_options[] = {
        {SAMPLES_OUT, required_argument, NULL,  0 },
        {BEACONS_OUT, required_argument, NULL,  0 },
        {0,         0,                 0,  0 }
    };
    while ((c = getopt_long(argc, argv, "hf:t:s:b:", long_options, &option_index )) != -1)
        switch (c)
    {
        case 0:
            if (strcmp(long_options[option_index].name, SAMPLES_OUT) == 0){
                opt.samples_out_path.assign(optarg);
            }
            if (strcmp(long_options[option_index].name, BEACONS_OUT) == 0){
                opt.beacons_out_path.assign(optarg);
            }
            break;
        case 'h':
            printHelp(lastComponent(argv[0]));
            abort();
        case 'f':
            opt.from.assign(optarg);
            break;
        case 't':
            opt.to.assign(optarg);
            break;
        case 's':
            opt.samples_path.assign(optarg);
            break;
        case 'b':
            opt.beacons_path.assign(optarg);
            break;
        default:
            abort();
    }
    return opt;
}


Location convertAnchorOrigin(Anchor from, Anchor to){
    LatLngConverter converter_from;
    LatLngConverter converter_to;
    converter_from.anchor(from);
    converter_to.anchor(to);
    Location loc(0,0,0,0);
    auto gl = converter_from.localToGlobal(loc);
    loc = converter_to.globalToLocal(gl);
    return loc;
}


Samples convertSamples(const Samples& samples, Anchor from, Anchor to){
    LatLngConverter converter_from;
    LatLngConverter converter_to;
    converter_from.anchor(from);
    converter_to.anchor(to);
    
    Samples samplesNew;
    for(auto& s: samples){
        Sample snew(s);
        auto loc = s.location();
        auto gl = converter_from.localToGlobal(loc);
        loc = converter_to.globalToLocal(gl);
        snew.location(loc);
        samplesNew.push_back(snew);
    }
    return samplesNew;
}

BLEBeacons convertBLEBeacons(const BLEBeacons& bleBeacons, Anchor from, Anchor to){
    LatLngConverter converter_from;
    LatLngConverter converter_to;
    converter_from.anchor(from);
    converter_to.anchor(to);
    
    BLEBeacons bleBeaconsNew;
    for(auto& b : bleBeacons){
        auto gl = converter_from.localToGlobal(b);
        auto local = converter_to.globalToLocal(gl);
        bleBeaconsNew.push_back(local);
    }
    return bleBeaconsNew;
}
int main(int argc, char * argv[]){
    if (argc <= 1) {
        printHelp(lastComponent(argv[0]));
        return 0;
    }
    
    Option opt = parseArguments(argc, argv);
    std::cout << "Convert coordinates" << std::endl;
    std::cout << "from anchor: " << opt.from << std::endl;
    std::cout << "to anchor: " << opt.to << std::endl;
    
    Anchor fromAnchor = parseJsonAnchor(opt.from);
    Anchor toAnchor = parseJsonAnchor(opt.to);
    
    std::cout << "Converted origin = " << convertAnchorOrigin(fromAnchor, toAnchor) << std::endl;
    
    if(strcmp(opt.samples_path.c_str(), "") != 0){
        std::cout << "samples_path=" << opt.samples_path << std::endl;
        std::ifstream ifs(opt.samples_path);
        if(ifs.is_open()){
            auto samples = DataUtils::csvSamplesToSamples(ifs);
            auto samplesNew = convertSamples(samples, fromAnchor, toAnchor);
            
            if(strcmp(opt.samples_out_path.c_str(),"") !=0){
                std::cout << "samples_out_path=" << opt.samples_out_path << std::endl;
            }
            std::ofstream ofs(opt.samples_out_path);
            if(ofs.is_open()){
                ofs << DataUtils::samplesToCsvSamples(samplesNew);
            }else{
                std::cout << DataUtils::samplesToCsvSamples(samplesNew);
            }
        }else{
            std::cerr << "File not found: " << opt.samples_path << std::endl;
        }
    }
    
    if(strcmp(opt.beacons_path.c_str(), "") != 0){
        std::cout << "beacons_path=" << opt.samples_path << std::endl;
        std::ifstream ifs(opt.beacons_path);
        if(ifs.is_open()){
            auto bleBeacons = DataUtils::csvBLEBeaconsToBLEBeacons(ifs);
            auto bleBeaconsNew = convertBLEBeacons(bleBeacons, fromAnchor, toAnchor);
            
            if(strcmp(opt.beacons_out_path.c_str(),"") !=0){
                std::cout << "beacons_out_path=" << opt.beacons_out_path << std::endl;
            }
            std::ofstream ofs(opt.beacons_out_path);
            if(ofs.is_open()){
                ofs << DataUtils::BLEBeaconsToCSV(bleBeaconsNew);
            }else{
                std::cout << DataUtils::BLEBeaconsToCSV(bleBeaconsNew);
            }
        }else{
            std::cerr << "File not found: " << opt.beacons_path << std::endl;
        }
    }

    return 0;
}
