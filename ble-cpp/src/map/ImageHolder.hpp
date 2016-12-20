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

#ifndef ImageHolder_hpp
#define ImageHolder_hpp

#include <stdio.h>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

namespace loc {

    enum ImageHolderMode{
        light=0,heavy=1
    };
    
    //Color struct is defined to keep color in RGB format.
    class Color{
    public:
        uint8_t r_, g_, b_;
        Color(uint8_t r, uint8_t g, uint8_t b);
        bool equals(const Color& c) const;
        bool operator==(const Color& right) const;
        bool operator<(const Color& right) const;
        int rgbcode() const;
    };
    
    namespace color{
        const Color black(0,0,0);
        const Color white(255, 255, 255);
        const Color red(255,0,0);
        const Color lime(0,255,0);
        const Color blue(0,0,255);
        const Color yellow(255, 255, 0);
        
        const Color green(0,128,0);
        const Color olive(128,128,0);
        const Color fuchsia(255,0,255);
        const Color silver(192,192,192);
        const Color aqua(0,255,255);
        const Color gray(128,128,128);
        const Color purple(128,128,128);
        const Color navy(0,0,128);
        const Color teal(0,128,128);
        const Color maroon(128,0,0);
    }
        
    class ImageHolder{
        std::string name_;
        cv::Mat mat_;
        
        class Impl;
        class ImplHeavy;
        class ImplLight;
        std::shared_ptr<Impl> impl;
        
        static ImageHolderMode mode_;
        
    public:
        class Point{
        public:
            Point() = default;
            ~Point() = default;
            Point(int x, int y){this->x=x; this->y=y;}
            int x;
            int y;
            static double distance(const Point& left, const Point& right);
        };
        using Points = std::vector<Point>;
        
        ImageHolder();
        ImageHolder(const std::string& filepath, const std::string& name);
        ~ImageHolder();
        
        static void setMode(ImageHolderMode mode);
        
        int rows() const;
        int cols() const;
        
        bool checkValid(int y, int x) const;
        Color get(int y, int x) const;
        
        void setUpIndexForColor(const Color& c);
        std::vector<Point> getPoints(const Color& c) const;
        Points findClosestPoints(const Color&c, const Point& p, int k=1) const;
    };
    
}
#endif /* ImageMap_hpp */
