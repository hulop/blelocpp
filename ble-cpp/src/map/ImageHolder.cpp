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
#include <opencv2/flann/flann.hpp>
#include <opencv2/opencv.hpp>

namespace loc{
    
    double ImageHolder::Point::distance(const Point& left, const Point& right){
        double x1 = left.x;
        double y1 = left.y;
        double x2 = right.x;
        double y2 = right.y;
        double sq = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
        double dist = std::sqrt(sq);
        return dist;
    }
    
    ImageHolderMode ImageHolder::mode_ = light;
    bool ImageHolder::precomputesIndex = false;
    
    Color::Color(uint8_t r, uint8_t g, uint8_t b){
        r_ = r;
        g_ = g;
        b_ = b;
    }
    
    static const std::vector<Color> colorList{
        color::white,
        color::black,
        color::red,
        color::lime,
        color::blue,
        color::yellow,
        color::green,
        color::aqua
        /*
        color::olive,
        color::fuchsia,
        color::silver,
        color::gray,
        color::purple,
        color::navy,
        color::teal,
        color::maroon
        */
    };
    
    static const std::vector<Color> colorsToIndices{
        color::red,
        color::lime,
        color::blue,
        color::yellow,
        color::green,
        color::aqua
    };
    
    bool Color::equals(const Color& c)const{
        if( this->b_ == c.b_
           && this->g_ == c.g_
           && this->r_ == c.r_
           ){
            return true;
        }else{
            return false;
        }
    }
    
    bool Color::operator==(const Color& right) const{
        return this->equals(right);
    }
    
    int Color::rgbcode() const{
        int t = 65536*this->r_ + 256*this->g_ + this->b_;
        return t;
    }
    
    bool Color::operator<(const Color& right) const{
        return this->rgbcode() < right.rgbcode();
    }
    
    bool ImageHolder::checkValid(int y, int x) const
    {
        if(0<=y && y<rows() && 0<=x && x<cols()){
            return true;
        }else{
            return false;
        }
    }
    
    class IndexWrapper: public cv::flann::Index{
    public:
        IndexWrapper(const IndexWrapper& idx) : Index(idx){};
        IndexWrapper(): Index(){}
        IndexWrapper(cv::InputArray data,
                     const cv::flann::IndexParams& params,
                     cvflann::flann_distance_t distType):
                    Index(data, params, distType){}
        ~IndexWrapper(){};
    };
    
    class ImageHolder::Impl{
    protected:
        std::map<Color, ImageHolder::Points> mColorPointsMap;
        
        std::map<Color, cv::Mat> mColorDataMap;
        mutable std::mutex mtx_;
        mutable std::map<Color, std::shared_ptr<IndexWrapper>> mColorIndexMap;
        
    public:
        virtual ~Impl() = default;
        virtual int rows() const = 0;
        virtual int cols() const = 0;
        virtual Color get(int y, int x) const = 0;
        virtual std::vector<Point> getPoints(const Color& c) const = 0;
        
        virtual void setUpIndices(){
            for(const Color& c: colorsToIndices){
                this->setUpIndexForColor(c);
            }
        }
        
        virtual void setUpIndexForColor(const Color& c){
            std::lock_guard<std::mutex> lock(mtx_);
            if(mColorPointsMap.count(c)==0){
                auto points = getPoints(c);
                int n = static_cast<int>(points.size());
                mColorPointsMap[c] = points;
                if(n>0){
                    cv::Mat data = cv::Mat::zeros(n, 2, CV_32FC1);
                    for(int j=0; j<n; j++){
                        data.at<float>(j,0) = points.at(j).x;
                        data.at<float>(j,1) = points.at(j).y;
                    }
                    mColorDataMap[c] = data;
                    if(ImageHolder::precomputesIndex){
                        mColorIndexMap[c] = std::make_shared<IndexWrapper>(mColorDataMap[c], cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
                    }
                }
            }
        }
        
        virtual Points findClosestPoints(const Color& c, const ImageHolder::Point& p, int k = 1) const{
            
            std::lock_guard<std::mutex> lock(mtx_);
            
            if(mColorPointsMap.count(c)==0){
                LocException ex("Index is not set for the input color");
                BOOST_THROW_EXCEPTION(ex);
            }
            
            const Points& points = mColorPointsMap.at(c);
            if(points.size()==0){
                return Points(0);
            }
            
            cv::Mat query = (cv::Mat_<float>(1,2) << p.x , p.y);
            std::vector<int> indices;
            std::vector<float> dists;
            
            {
                if(ImageHolder::precomputesIndex){
                    if(mColorIndexMap.count(c)==0){
                        cv::Mat data = mColorDataMap.at(c);
                        mColorIndexMap[c] = std::make_shared<IndexWrapper>(data, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
                    }else{
                        mColorIndexMap.at(c)->knnSearch(query, indices, dists, k);
                    }
                }else{
                    cv::Mat data = mColorDataMap.at(c);
                    cv::flann::Index idx(data, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
                    idx.knnSearch(query, indices, dists, k);
                }
            }
            
            Points psRet(k);
            for(int i=0; i<k; i++){
                int index = indices.at(0);
                Point pret = points.at(index);
                psRet[i] = pret;
            }
            return psRet;
        }
    };
    
    class ImageHolder::ImplHeavy : public ImageHolder::Impl{
        std::string name_;
        cv::Mat mat_;
        
    public:
        ImplHeavy(){}
        ImplHeavy(const std::string& filepath, const std::string& name){
            name_ = name;
            mat_ = cv::imread(filepath);
            if(mat_.empty()){
                LocException ex("Failed to read the image file at " + filepath);
                BOOST_THROW_EXCEPTION(ex);
            }
            
            this->setUpIndices();
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
            uint8_t b = vec[0];
            uint8_t g = vec[1];
            uint8_t r = vec[2];
            Color color(r,g,b);
            return color;
        }
        
        std::vector<Point> getPoints(const Color& c) const{
            cv::Mat binary;
            cv::inRange(mat_, cv::Scalar(c.b_, c.g_, c.r_), cv::Scalar(c.b_, c.g_, c.r_), binary);
            cv::Mat idx;
            cv::findNonZero(binary, idx);
            std::vector<Point> points;
            for(int i=0; i<idx.total(); i++){
                Point p;
                p.x = idx.at<cv::Point>(i).x;
                p.y = idx.at<cv::Point>(i).y;
                points.push_back(p);
            }
            return points;
        }
    };
    
    class ImageHolder::ImplLight : public ImageHolder::Impl{
        // variables to be serialized
        uint32_t cereal_class_version = 0;
        std::string name_;
        Eigen::SparseMatrix<uint8_t> mat_;
        
        // not to be serialized (created)
        boost::bimap<Color, uint8_t> colorIntBM;
        
    public:
        ImplLight(){
            using bm_type = boost::bimap<Color, uint8_t>;
            for(int i=0; i<colorList.size(); i++){
                const Color& c = colorList.at(i);
                colorIntBM.insert( bm_type::value_type(c, i));
            }
        }
        
        ImplLight(const std::string& filepath, const std::string& name) : ImplLight(){
            
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
            mat_.makeCompressed();
            
            this->setUpIndices();
        }
        ~ImplLight() = default;
        
        
        uint8_t colorToUint8(const Color& color) const{
            for(int i=0; i<colorList.size(); i++){
                const Color& c = colorList.at(i);
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
        
        /*
        uint8_t colorToUint8BM(const Color& color) const{
            if(colorIntBM.left.count(color) != 0){
                return colorIntBM.left.at(color);
            }else{
                return colorIntBM.left.at(color::white);
            }
        }
        
        Color uint8ToColorBM(uint8_t i) const{
            if(colorIntBM.right.count(i) != 0){
                return colorIntBM.right.at(0);
            }else{
                return colorIntBM.right.at(i);
            }
        }
        */
        
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
        
        std::vector<Point> getPoints(const Color& c) const{
            Points points;
            uint8_t code_q = colorToUint8(c);
            for(int k=0; k<mat_.outerSize(); k++){
                auto iter = Eigen::SparseMatrix<uint8_t>::InnerIterator(mat_, k);
                for(; iter; ++iter){
                    uint8_t code = iter.value();
                    if(code == code_q){
                        Point p;
                        p.x = iter.col();
                        p.y = iter.row();
                        points.push_back(p);
                    }
                }
            }
            return points;
        }
        
        
        template<class Archive>
        void save(Archive & ar) const
        {
            ar(cereal::make_nvp("cereal_class_version", cereal_class_version));
            ar(cereal::make_nvp("name_", name_));
            ar(cereal::make_nvp("mat_", mat_));
            //saveColorMat(ar);
        }
        
        template<class Archive>
        void load(Archive & ar)
        {
            ar(cereal::make_nvp("cereal_class_version", cereal_class_version));
            ar(cereal::make_nvp("name_", name_));
            ar(cereal::make_nvp("mat_", mat_));
            //loadColorMat(ar);
        }
        
        class BinarySparseMatrix{
        public:
            uint8_t value;
            Eigen::SparseMatrix<uint8_t> X;
            
            template<class Archive>
            void save(Archive& ar) const{
                // column-major
                typename Eigen::SparseMatrix<uint8_t>::Index rows = X.rows(), cols = X.cols(), nonZeros = X.nonZeros(), outerSize = X.outerSize(), innerSize = X.innerSize();
                ar(CEREAL_NVP(rows));
                ar(CEREAL_NVP(cols));
                ar(CEREAL_NVP(nonZeros));
                ar(CEREAL_NVP(outerSize));
                ar(CEREAL_NVP(innerSize));
                
                std::vector<int> outerStarts(X.outerIndexPtr(), X.outerIndexPtr() + outerSize + 1);
                std::vector<int> innerIndices(X.innerIndexPtr(), X.innerIndexPtr() + nonZeros);
                
                ar(CEREAL_NVP(value));
                ar(CEREAL_NVP(outerStarts));
                ar(CEREAL_NVP(innerIndices));
            }
            
            template<class Archive>
            void load(Archive& ar){
                // column-major
                typename Eigen::SparseMatrix<uint8_t>::Index rows = X.rows(), cols = X.cols(), nonZeros = X.nonZeros(), outerSize = X.outerSize(), innerSize = X.innerSize();
                ar(CEREAL_NVP(rows));
                ar(CEREAL_NVP(cols));
                ar(CEREAL_NVP(nonZeros));
                ar(CEREAL_NVP(outerSize));
                ar(CEREAL_NVP(innerSize));
                
                std::vector<int> outerStarts;
                std::vector<int> innerIndices;
                //std::vector<uint8_t> values;
                
                ar(CEREAL_NVP(value));
                ar(CEREAL_NVP(outerStarts));
                ar(CEREAL_NVP(innerIndices));
                
                X.resize(rows, cols);
                X.makeCompressed();
                X.resizeNonZeros(nonZeros);
                
                std::copy(outerStarts.begin(), outerStarts.end(), X.outerIndexPtr());
                std::copy(innerIndices.begin(), innerIndices.end(), X.innerIndexPtr());
                for(int i=0; i<nonZeros; i++){
                    X.valuePtr()[i] = value;
                }
                X.finalize();
            }
        };
        
        class LayeredSparseMatrix{
        public:
            int rows;
            int cols;
            std::vector<BinarySparseMatrix> bsMats;
            
            template<class Archive>
            void serialize(Archive& ar){
                ar(cereal::make_nvp("rows", rows));
                ar(cereal::make_nvp("cols", cols));
                ar(cereal::make_nvp("mats", bsMats));
            }
        };
        
        template<class Archive>
        void saveColorMat(Archive & ar) const
        {
            int rows = mat_.rows();
            int cols = mat_.cols();
            
            typedef Eigen::Triplet<uint8_t> Triplet;
            std::map<uint8_t, std::vector<Triplet>> colorTriplets;
            
            for (int k=0; k<mat_.outerSize(); ++k){
                for (Eigen::SparseMatrix<uint8_t>::InnerIterator it(mat_,k); it; ++it){
                    uint8_t val = it.value();
                    if(val==0){// stores only non-zero elements
                        continue;
                    }
                    if(colorTriplets.count(val) == 0){
                        colorTriplets[val] = std::vector<Triplet>();
                    }
                    colorTriplets[val].push_back(Triplet(it.row(), it.col(), val));
                }
            }
            
            std::vector<BinarySparseMatrix> bsMats;
            for(auto iter=colorTriplets.begin(); iter!=colorTriplets.end(); iter++){
                uint8_t value = iter->first;
                auto triplets = iter->second;
                Eigen::SparseMatrix<uint8_t> sMat;
                sMat.resize(rows, cols);
                sMat.setFromTriplets(triplets.begin(), triplets.end());
                sMat.makeCompressed();
                BinarySparseMatrix bsMat;
                bsMat.value = value;
                bsMat.X = sMat;
                bsMats.push_back(bsMat);
            }
            
            LayeredSparseMatrix lsm;
            lsm.rows = rows;
            lsm.cols = cols;
            lsm.bsMats = bsMats;
            
            ar(cereal::make_nvp("layered", lsm));
        }
        
        
        template<class Archive>
        void loadColorMat(Archive & ar)
        {
            LayeredSparseMatrix lsm;
            ar(cereal::make_nvp("layered", lsm));
    
            int rows = lsm.rows;
            int cols = lsm.cols;
            std::vector<BinarySparseMatrix> bsMats = lsm.bsMats;
            
            typedef Eigen::Triplet<uint8_t> Triplet;
            std::vector<Triplet> triplets;
            for(auto bsMat: bsMats){
                auto value = bsMat.value;
                auto X = bsMat.X;
                for (int k=0; k<X.outerSize(); ++k){
                    for (Eigen::SparseMatrix<uint8_t>::InnerIterator it(X,k); it; ++it){
                        triplets.push_back(Triplet(it.row(), it.col(), value));
                    }
                }
            }
            
            Eigen::SparseMatrix<uint8_t> sMat;
            sMat.resize(rows, cols);
            sMat.setFromTriplets(triplets.begin(), triplets.end());
            sMat.makeCompressed();
            
            mat_ = sMat;
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
    
    ImageHolder::ImageHolder(const std::string& filepath, const std::string& name){
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
    
    void ImageHolder::setUpIndexForColor(const loc::Color &c){
        impl->setUpIndexForColor(c);
    }
    
    std::vector<ImageHolder::Point> ImageHolder::getPoints(const Color& c) const{
        return impl->getPoints(c);
    }
    
    ImageHolder::Points ImageHolder::findClosestPoints(const Color& c, const ImageHolder::Point& p, int k) const{
        return impl->findClosestPoints(c, p, k);
    }
    
    void ImageHolder::setPrecomputesIndex(bool precomputesIdx){
        ImageHolder::precomputesIndex = precomputesIdx;
    }
    
    
    template<class Archive>
    void ImageHolder::serialize(Archive & ar, std::uint32_t const version)
    {
        ar(cereal::make_nvp("mode", mode_));
        ar(cereal::make_nvp("precomputesIndex", precomputesIndex));
        
        if(mode_ == ImageHolderMode::light){
            auto implLight = std::dynamic_pointer_cast<ImplLight>(impl);
            ar(cereal::make_nvp("implLight", *implLight));
        }else if(mode_ == ImageHolderMode::heavy){
            LocException ex("Serialization of ImageHolder::ImplHeavy is currently unsupported.");
            BOOST_THROW_EXCEPTION(ex);
        }
    }
    
    template void ImageHolder::serialize<cereal::JSONOutputArchive>(cereal::JSONOutputArchive& ar, std::uint32_t const version);
    template void ImageHolder::serialize<cereal::JSONInputArchive>(cereal::JSONInputArchive& ar, std::uint32_t const version);
    template void ImageHolder::serialize<cereal::PortableBinaryOutputArchive>(cereal::PortableBinaryOutputArchive& ar, std::uint32_t const version);
    template void ImageHolder::serialize<cereal::PortableBinaryInputArchive>(cereal::PortableBinaryInputArchive& ar, std::uint32_t const version);
}
