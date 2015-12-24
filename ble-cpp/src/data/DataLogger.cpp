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

#include "DataLogger.hpp"

namespace loc{
    
    std::shared_ptr<DataLogger> DataLogger::instance;
    
    void DataLogger::resultDirectory(const std::string& resultDirectory){
        mResultDirectory = resultDirectory;
    }
    
    void DataLogger::log(const std::string& fileName, const std::string& loggedString){
        std::string resultPath = mResultDirectory + "/" + fileName;
        std::ofstream ofs(resultPath);
        ofs << loggedString << std::endl;
        ofs.close();
    }
    
    void DataLogger::createInstance(const std::string resultDirectory){
        instance = std::shared_ptr<DataLogger>(new DataLogger());
        instance->resultDirectory(resultDirectory);
    }
    
    std::shared_ptr<DataLogger>& DataLogger::getInstance(){
        return instance;
    }
    
}