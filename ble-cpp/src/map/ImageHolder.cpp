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

#include <Eigen/SparseCore>
#include <boost/bimap.hpp>
#include "ImageHolder.hpp"
#include "LocException.hpp"

namespace loc{
    
    ImageHolderMode ImageHolder::mode_ = light;
    
    Color::Color(int r, int g, int b){
        r_ = r;
        g_ = g;
        b_ = b;
    }
    
    static std::vector<Color> colorList;
    class ColorListInitializer{
    public:
        ColorListInitializer(){
            colorList.push_back(color::white);
            colorList.push_back(color::black);
            colorList.push_back(color::red);
            colorList.push_back(color::green);
            colorList.push_back(color::blue);
            colorList.push_back(color::yellow);
        }
    };
    static ColorListInitializer colorListInitializer;
    
    bool Color::equals(Color c)const{
        if( this->b_ == c.b_
           && this->g_ == c.g_
           && this->r_ == c.r_
           ){
            return true;
        }else{
            return false;
        }
    }
    
    bool ImageHolder::checkValid(int y, int x) const
    {
        if(0<=y && y<rows() && 0<=x && x<cols()){
            return true;
        }else{
            return false;
        }
    }
    
    class ImageHolder::Impl{
    public:
        virtual ~Impl() = default;
        virtual int rows() const = 0;
        virtual int cols() const = 0;
        virtual Color get(int y, int x) const = 0;
    };
    
    class ImageHolder::ImplHeavy : public ImageHolder::Impl{
        std::string name_;
        cv::Mat mat_;
        
    public:
        ImplHeavy(){}
        ImplHeavy(std::string filepath, const std::string name){
            name_ = name;
            mat_ = cv::imread(filepath);
            if(mat_.empty()){
                LocException ex("Failed to read the image file at " + filepath);
                BOOST_THROW_EXCEPTION(ex);
            }
        }
        ~ImplHeavy() = default;
        
        int rows() const{
            return mat_.rows;
        }
        int cols() const{
            return mat_.rows;
        }
        Color get(int y, int x) const{
            cv::Vec3b vec = mat_.at<cv::Vec3b>(y, x);
            int b = vec[0];
            int g = vec[1];
            int r = vec[2];
            Color color(r,g,b);
            return color;
        }
    };
    
    class ImageHolder::ImplLight : public ImageHolder::Impl{
        std::string name_;
        Eigen::SparseMatrix<uint8_t> mat_;
    public:
        ImplLight(){}
        ImplLight(std::string filepath, const std::string name){
            
            typedef Eigen::Triplet<uint8_t> Triplet;
            
            name_ = name;
            int rows, cols;
            std::vector<Triplet> tripletList;
            {
                cv::Mat image = cv::imread(filepath);
                cols = image.cols;
                rows = image.rows;
                for(int y=0; y<image.rows; y++){
                    for(int x=0; x<image.cols; x++){
                        cv::Vec3b vec = image.at<cv::Vec3b>(y, x);
                        int b = vec[0];
                        int g = vec[1];
                        int r = vec[2];
                        Color color(r,g,b);
                        uint8_t code = colorToUint8(color);
                        if(code!=0){
                            tripletList.push_back(Triplet(y,x,code));
                        }
                    }
                }
                mat_ = Eigen::SparseMatrix<uint8_t>(rows, cols);
                image.release();
            }
            
            mat_.setFromTriplets(tripletList.begin(), tripletList.end());
        }
        ~ImplLight() = default;
        
        uint8_t colorToUint8(Color color) const{
            for(int i=0; i<colorList.size(); i++){
                Color c = colorList.at(i);
                if(color.equals(c)){
                    return i;
                }
            }
            return 0;
        }
        
        Color uint8ToColor(uint8_t i) const{
            if(i<colorList.size()){
                return colorList.at(i);
            }else{
                return colorList.at(0);
            }
        }
        
        int rows() const{
            return mat_.rows();
        }
        int cols() const{
            return mat_.cols();
        }
        
        Color get(int y, int x) const{
            uint8_t code = mat_.coeff(y,x);
            Color color = uint8ToColor(code);
            return color;
        }
    };
    
    ImageHolder::ImageHolder(){
        if(mode_ == light){
            impl.reset(new ImplLight());
        }else if (mode_ == heavy){
            impl.reset(new ImplHeavy());
        }else{
            BOOST_THROW_EXCEPTION(LocException("Unknown ImageHolderMode."));
        }
    }
    
    ImageHolder::ImageHolder(std::string filepath, const std::string name){
        if(mode_ == light){
            std::cout << "ImageHolder::ImplLight is instantiated." << std::endl;
            impl.reset(new ImplLight(filepath, name));
        }else if (mode_ == heavy){
            std::cout << "ImageHolder::ImplHeavy is instantiated." << std::endl;
            impl.reset(new ImplHeavy(filepath, name));
        }else{
            BOOST_THROW_EXCEPTION(LocException("Unknown ImageHolderMode."));
        }
    }
    
    ImageHolder::~ImageHolder(){}
    
    void ImageHolder::setMode(ImageHolderMode mode){
        mode_ = mode;
    }
    
    int ImageHolder::rows() const{
        return impl->rows();
    }
    
    int ImageHolder::cols() const{
        return impl->cols();
    }
    
    Color ImageHolder::get(int y, int x) const{
        return impl->get(y, x);
    }
}
