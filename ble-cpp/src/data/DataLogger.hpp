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

#ifndef DataLogger_hpp
#define DataLogger_hpp

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <memory>
#ifdef ANDROID_STL_EXT
#include "string_ext.hpp"
#endif /* ANDROID_STL_EXT */

namespace loc{
    
    // Singleton class
    // Data are logged only when the instance is created.
    class DataLogger{
        
    private:
        static std::shared_ptr<DataLogger> instance;
        std::string mResultDirectory;
        
        DataLogger() = default;
        
    public:
        
        ~DataLogger() = default;
        
        void resultDirectory(const std::string& resultDirectory);
        void log(const std::string& fileName, const std::string& loggedString);
        static void createInstance(const std::string resultDirectory);
        static std::shared_ptr<DataLogger>& getInstance();
    };
}


#endif /* DataLogger_hpp */
