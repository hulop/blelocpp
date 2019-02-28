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

#ifndef GaussianProcessLight_hpp
#define GaussianProcessLight_hpp

#include <iostream>
#include <limits>
#include <Eigen/Dense>

#include "KernelFunction.hpp"
#include "GaussianProcess.hpp"
#include "SerializeUtils.hpp"

namespace loc{
    
    enum KNLType{
        KNLALL,
        KNLCLUSTERED
    };
    
    class GaussianProcessLight : public GaussianProcess{
        
    private:
        // variables to be serialized
        std::vector<GaussianProcess> LGPs_;     //Local Gaussian Processes
        std::vector<Eigen::VectorXd> centers_;  //center for each LGP
        
        double sigmaN_ = 1.0;
        GaussianKernel gaussianKernel_;
        KNLType knlType;
        double overlapScale = 0.001;
        
    public:
        static const int N_FEATURES = 4;
        constexpr static const double MIN_DENOMINATOR = std::numeric_limits<double>::min() * 1e+16;

        GaussianProcessLight() = default;
        
        enum ClusteringType{
            GREEDY,
            KMEANS
        };
        ClusteringType clType = KMEANS;
        bool usesOverlap = true;
        int mLocalsMixed_ = 3; // designed value
        
        // A function for serealization
        template<class Archive>
        void serialize(Archive& ar){
            ar(CEREAL_NVP(LGPs_));
            ar(CEREAL_NVP(centers_));
            ar(CEREAL_NVP(sigmaN_));
            ar(CEREAL_NVP(gaussianKernel_));
            
            std::vector<std::string> names;
            try{
                ar(CEREAL_NVP(mLocalsMixed_));
            }catch(cereal::Exception& e){
                names.push_back("mLocalsMixed_");
            }
            
            if(0<names.size()){
                std::stringstream ss;
                ss << "[notice] parameters not found: ";
                for(const auto& name: names){
                    ss << name << ",";
                }
                std::cerr << ss.str() << std::endl;
            }
        }
        
        GaussianProcessLight& sigmaN(double sigmaN){
            sigmaN_ = sigmaN;
            return *this;
        }
        
        double sigmaN() const{
            return sigmaN_;
        }
        
        void setKNLType(KNLType knl){
            knlType = knl;
        }
        
        void setOverlapScale(double os){
            overlapScale = os;
        }
        
        //Calculate maximum cluster size based on max complexity of the function predict()
        static const size_t MAX_CLUSTER_SIZE(const size_t MAX_COMPLEXITY,
                                             const size_t MAX_N_OVERLAP,
                                             const size_t N_LOCALS_MIXED) {
            const size_t k = MAX_N_OVERLAP;     //max # cluster overlaps for a sample
            const size_t M = N_LOCALS_MIXED;    //# local models mixed in prediction
            const size_t MAX_AMP = (std::min(k, M) + 1) * std::max(k, M);
            const size_t MAX_CLUSTER_SIZE = MAX_COMPLEXITY / MAX_AMP;
            
            const size_t MIN_FEASIBLE_CLUSTER_SIZE = 50;
            if (MAX_CLUSTER_SIZE < MIN_FEASIBLE_CLUSTER_SIZE) {
                std::cout << "WARNING: MAX_CLUSTER_SIZE=" << MAX_CLUSTER_SIZE << " is less than MIN_FEASIBLE_CLUSTER_SIZE=" << MIN_FEASIBLE_CLUSTER_SIZE << std::endl;
            }
            return MAX_CLUSTER_SIZE;
        }

        GaussianProcessLight& fit(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y)
        {
            //Clustering samples
            const size_t TARGET_N_CLUSTER = 1 + (X.rows() / MAX_CLUSTER_SIZE(800, 3, 3));
            CentroidBasedClusteringResult cr;
            if(clType==GREEDY){
                cr = clusteringGreedyWithMagicThreshold(X, Y, 1e-8);
            }else if(clType==KMEANS){
                cr = kMeansClustering(X, Y, TARGET_N_CLUSTER);
            }
            
            cr.printSummary();

            if(usesOverlap){
                //Improve the clusters
                improveWithOverlap(cr, overlapScale, X, Y);
                cr.printSummary();
            }
                
            std::cout << "clustered into " << cr.nCluster() << " local models" << std::endl;
            
            centers_ = cr.centers;
            
            //Get local models by cluster
            for (auto k=0; k < cr.nCluster(); k++) {
                GaussianProcess gp;
                gp.setAsSparse(this->asSparse_);
                gp.sigmaN(sigmaN_);
                gp.gaussianKernel(gaussianKernel_);
                
                gp.fit(cr.XC[k], cr.YC[k]);
                
                LGPs_.push_back(gp);
            }
            
            return *this;
        }
        
        double predict(double x[], int index) const 
        {
            std::vector<int> indices(1, index);
            std::vector<double> ypreds = predict(x, indices);
            return ypreds.at(0);
        }
        
        //TODO change return type: Eigen::VectorXd would be better
        std::vector<double> predict(double x[], const std::vector<int>& indices) const
        {
            const size_t M = mLocalsMixed_;
            const size_t n = centers_.size();
            
            std::vector<double> weights(n);
            for (size_t i=0; i < n; ++i) {
                const double* c = centers_.at(i).data();
                weights[i] = gaussianKernel_.computeKernel(x, c);
            }
            
            //indices of k-nearest (=top-k weight) neigbors
            std::vector<size_t> neighbors = top_k(weights, std::min(M, n));
            
            Eigen::VectorXd sum_wy = Eigen::VectorXd::Zero(indices.size());
            double sum_w = 0.0;
            for (auto m : neighbors) {
                double w = weights.at(m);
                std::vector<double> tmp = LGPs_.at(m).predict(x, indices);
                Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd>(tmp.data(), indices.size());
                sum_wy += w * y;
                sum_w  += w;
            }
            
            Eigen::VectorXd y_hat = sum_wy;
            if (sum_w > MIN_DENOMINATOR) {
                y_hat /= sum_w;
            } else {
                std::vector<double> top1ypred = LGPs_.at(neighbors.at(0)).predict(x, indices);
                y_hat = Eigen::Map<Eigen::VectorXd>(top1ypred.data(), indices.size());
//                std::cout << "WARN: sum_w~=0 in predict() with " << n << " LGPs"
//                          << " >> predicted only with the nearest local model."<< std::endl;
            }
            std::vector<double> ypreds(y_hat.data(), y_hat.data() + indices.size());
            return ypreds;
        }

        /**
         * Estimate parameters as preparation
         */
        void fitCV(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives)
        {
            if(knlType==KNLALL){
                this->fitGlobalCV(X, Y, Actives);
            }else if(knlType==KNLCLUSTERED){
                this->fitFixedClusterCV(X, Y, Actives);
            }
        }
        
        void fitGlobalCV(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& Actives)
        {
            GaussianProcess gp;
            gp.sigmaN(sigmaN_);
            gp.gaussianKernel(gaussianKernel_);
            
            // estimate parameters using GaussianProcess::fitCV
            gp.fitCV(X, Y, Actives);
            
            // set estimated parameters to this
            sigmaN_ = gp.sigmaN();
            gaussianKernel_ = gp.gaussianKernel();
            
            this->fit(X, Y);
        }
        
        void fitFixedClusterCV(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const Eigen::MatrixXd& A)
        {
            // Set the kernel with hardcoded parameters (set large clusters)
            GaussianKernel::Parameters gkParams;
            gkParams.sigma_f = 2.0;   //1.0
            gkParams.lengthes[0] = 7.0;  //3.0
            gkParams.lengthes[1] = 7.0;
            gkParams.lengthes[2] = 7.0;
            gkParams.lengthes[3] = 0.01;
            GaussianKernel gKernel(gkParams);
            gaussianKernel_ = gKernel;
            
            // Change the number of mixed local models to reduce the computation time
            mLocalsMixed_ = 1;
            
            //Clustering samples
            const size_t TARGET_N_CLUSTER = 1 + (X.rows() / MAX_CLUSTER_SIZE(800, 3, 3));
            CentroidBasedClusteringResult cr;
            if(clType==GREEDY){
                cr = clusteringGreedyWithMagicThreshold(X, Y, A, 1e-8);
            }else if(clType==KMEANS){
                cr = kMeansClustering(X, Y, A, TARGET_N_CLUSTER);
            }
            
            cr.printSummary();
            std::cout << "overlapScale " << overlapScale << std::endl;
            
            if(usesOverlap){
                //Improve the clusters
                improveWithOverlap(cr, overlapScale, X, Y, A);
                cr.printSummary();
            }
            
            std::cout << "clustered into " << cr.nCluster() << " local models" << std::endl;
            
            centers_ = cr.centers;
            
            // set parameters for clustered CV
            GaussianProcessParameterSet gpParamSet;
            //gpParamSet.sigmaFs = sigmaFsClustered;
            //gpParamSet.lengthes = lengthesClustered;
            //gpParamSet.lengthFloors = lengthFloorsClustered;
            //gpParamSet.sigmaNs = sigmaNsClustered;
            
            //Get local models by cluster
            for (auto k=0; k < cr.nCluster(); k++) {
                GaussianProcess gp;
                gp.setAsSparse(this->asSparse_);
                gp.sigmaN(sigmaN_);
                gp.gaussianKernel(gaussianKernel_);
                gp.gaussianProcessParameterSet(gpParamSet);
                
                gp.fitCV(cr.XC[k], cr.YC[k], cr.AC[k]); // Estimate hyper parameters
                
                LGPs_.push_back(gp);
            }
        }
        
//        Eigen::VectorXd predictVarianceF(double x[]) const;
//        Eigen::VectorXd predictVarianceF(const Eigen::VectorXd& kstar) const{;
//        double computeLogLikelihood(double x[], const Eigen::VectorXd& y) const;

    private:
        class CentroidBasedClusteringResult{
        public:
            std::vector<Eigen::MatrixXd> XC;
            std::vector<Eigen::MatrixXd> YC;
            std::vector<Eigen::MatrixXd> AC;
            std::vector<Eigen::VectorXd> centers = {};
            size_t nCluster() const { return centers.size(); }
            void printSummary() const;
            void printAll() const;
        };
        
        /**
         * k-means++ clustering
         */
        CentroidBasedClusteringResult kMeansClustering(const Eigen::MatrixXd& X,
                                                       const Eigen::MatrixXd& Y,
                                                       const size_t TARGET_N_CLUSTER) const;
        
        /**
         * k-means++ clustering with Actives matrix
         */
        CentroidBasedClusteringResult kMeansClustering(const Eigen::MatrixXd& X,
                                                       const Eigen::MatrixXd& Y,
                                                       const Eigen::MatrixXd& A,
                                                       const size_t TARGET_N_CLUSTER) const;
        
        /**
         * Greedy clustering 
         */
        CentroidBasedClusteringResult clusteringGreedyWithMagicThreshold(const Eigen::MatrixXd& X,
                                                                         const Eigen::MatrixXd& Y,
                                                                         const double MAGIC_W_THRESHOLD) const;
        
        /**
         * Greedy clustering with Actives matrix
         */
        CentroidBasedClusteringResult clusteringGreedyWithMagicThreshold(const Eigen::MatrixXd& X,
                                                                         const Eigen::MatrixXd& Y,
                                                                         const Eigen::MatrixXd& A,
                                                                         const double MAGIC_W_THRESHOLD) const;
        
        void improveWithOverlap(CentroidBasedClusteringResult& cr,
                                const double OVERLAP_SCALE,
                                const Eigen::MatrixXd& X,
                                const Eigen::MatrixXd& Y) const;
        
        void improveWithOverlap(CentroidBasedClusteringResult& cr,
                                const double OVERLAP_SCALE,
                                const Eigen::MatrixXd& X,
                                const Eigen::MatrixXd& Y,
                                const Eigen::MatrixXd& A ) const;
        
    public:
        // TODO move to an appropriate util class
        static std::vector<size_t> top_k(const std::vector<double>& values, const size_t k)
        {
            assert(k <= values.size());
            size_t n = values.size();
            std::vector<size_t> labels(n);
            for (auto i=0; i < n; ++i) { labels[i] = i; }
            
            auto comp = [&values](size_t i, size_t j) { return values[i] > values[j]; };
            
            if (n == k) {
                std::sort(labels.begin(), labels.end(), comp);
                return labels;
            }
            
            assert(n > k);  //need +1 elements to make heap
            auto first = labels.begin(), last = labels.begin() + k, end = labels.end();
            std::make_heap(first, last + 1, comp);
            std::pop_heap(first, last + 1, comp);
            for (auto it = last + 1; it != end; it++) {
                if (values[*it] <= values[*first]) {
                    continue;
                } else {
                    *last = *it;
                    std::pop_heap(first, last + 1, comp);
                }
            }
            std::sort_heap(first, last, comp);
            return std::vector<size_t>{first, last};
        }
        
//    private:
//        CentroidBasedClusteringResult aggregativeClustering(const Eigen::MatrixXd& X,
//                                                            const Eigen::MatrixXd& Y,
//                                                            const size_t TARGET_N_CLUSTER) const
//        class CentroidBasedCluster{
//        public:
//            CentroidBasedCluster(size_t i, const Eigen::MatrixXd& X) {
//                labels.push_back(i);
//                center = X.row(i);
//            };
//            std::vector<size_t> labels;
//            Eigen::VectorXd center;
//            void merge(const CentroidBasedCluster& that) {
//                std::copy(that.labels.begin(),that.labels.end(),std::back_inserter(this->labels));
//                std::sort(this->labels.begin(), this->labels.end());
//                double nis = this->labels.size();
//                double nat = that.labels.size();
//                this->center = (nis * this->center + nat * that.center) / (nis + nat);
//            }
//        };

    };
}

#endif /* GaussianProcessLight_hpp */
