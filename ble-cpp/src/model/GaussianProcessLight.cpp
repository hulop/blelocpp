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

#include <cfloat>
#include <random>

#include "GaussianProcessLight.hpp"

void loc::GaussianProcessLight::CentroidBasedClusteringResult::printSummary() const {
    for (auto i=0; i < nCluster(); i++) {
        std::cout << "cluster_" << i << ": " << centers[i].transpose() << ", XCrows=" << XC[i].rows() << std::endl;
    }
}
void loc::GaussianProcessLight::CentroidBasedClusteringResult::printAll() const {
    std::cout << "CLUSTER_ID,X,Y,Z,F," << std::endl;
    for (auto i=0; i < nCluster(); i++) {
        for (auto j=0; j < XC.at(i).rows(); j++) {
            std::cout << i << ",";
            for (auto k=0; k < XC.at(i).cols(); k++) {
                std::cout << XC.at(i)(j,k) << ",";
            }
            std::cout << std::endl;
        }
    }
}

loc::GaussianProcessLight::CentroidBasedClusteringResult
loc::GaussianProcessLight::kMeansClustering(const Eigen::MatrixXd& X,
                                            const Eigen::MatrixXd& Y,
                                            const size_t TARGET_N_CLUSTER) const
{
    assert(X.rows()==Y.rows());
    assert(X.cols()==N_FEATURES);
    assert(X.rows()>=TARGET_N_CLUSTER);
    std::cout << "TARGET_N_CLUSTER=" << TARGET_N_CLUSTER << std::endl;
    
    std::vector<Eigen::VectorXd> centers;
    
    //choose initial centers
    std::vector<Eigen::VectorXd> lefts(X.rows());
    for (auto i=0; i < X.rows(); i++) { lefts.at(i) = X.row(i); }
    
    std::mt19937 mt;
    for (size_t k=0; k < TARGET_N_CLUSTER; k++) {
        std::deque<double> dists(1, 0.0);
        for (auto x : lefts) {
            double min_d = DBL_MAX;
            for (auto c : centers) {
                min_d = std::min(min_d, gaussianKernel_.sqsum(x.data(), c.data()));
            }
            dists.push_back(dists.back() + min_d);
        }
        dists.pop_front();
        assert(dists.size() == lefts.size());
        std::uniform_real_distribution<> rand(0.0, dists.back());
        const double oracle = rand(mt);
        const auto it_chosen = std::lower_bound(dists.begin(), dists.end(), oracle);
        const size_t idx_chosen = std::distance(dists.begin(), it_chosen);
        centers.push_back(lefts.at(idx_chosen));
        lefts.erase(lefts.begin() + idx_chosen);
        assert(centers.size() + lefts.size() == X.rows());
    }
    assert(centers.size() == TARGET_N_CLUSTER);
    
    //k-means
    std::vector<std::vector<size_t>> labels(TARGET_N_CLUSTER);
    const size_t MAX_ITERATION = 32;
    for (auto r=0; r < MAX_ITERATION; r++) {
        //clear previous labels
        for (auto i=0; i<labels.size(); i++) { labels.at(i).clear(); }
        
        //assign each sample to the nearest cluster
        double sqdist_sum = 0.0;
        for (auto i=0; i < X.rows(); i++) {
            std::vector<double> d;
            for (auto c : centers) {
                Eigen::VectorXd x = X.row(i);
                d.push_back(gaussianKernel_.sqsum(x.data(), c.data()));
            }
            const auto it_nearest = std::min_element(d.begin(), d.end());
            sqdist_sum += *it_nearest;
            const size_t iNearestCluster = std::distance(d.begin(), it_nearest);
            labels.at(iNearestCluster).push_back(i);
        }
        std::cout << "sqdist_sum[" << r << "]= " << sqdist_sum << std::endl;
        for (auto l : labels) { std::cout << l.size() << ","; }
        std::cout << std::endl;
        
        //calculate cluster centers
        for (auto k=0; k < labels.size(); k++) {
            const auto kth_label = labels.at(k);
            Eigen::MatrixXd Xnext(kth_label.size(), X.cols());
            for (auto i=0; i < kth_label.size(); i++) {
                Xnext.row(i) = X.row(kth_label.at(i));
            }
            centers.at(k) = Xnext.colwise().mean();
        }
    }
    
    CentroidBasedClusteringResult res;
    res.centers = centers;
    for (auto k=0; k < labels.size(); k++) {
        const auto kth_label = labels.at(k);
        Eigen::MatrixXd Xnext(kth_label.size(), X.cols());
        Eigen::MatrixXd Ynext(kth_label.size(), Y.cols());
        for (auto i=0; i < kth_label.size(); i++) {
            Xnext.row(i) = X.row(kth_label.at(i));
            Ynext.row(i) = Y.row(kth_label.at(i));
        }
        res.XC.push_back(Xnext);
        res.YC.push_back(Ynext);
    }
    //need to remove empty clusters?
    
    return res;
}

loc::GaussianProcessLight::CentroidBasedClusteringResult
loc::GaussianProcessLight::kMeansClustering(const Eigen::MatrixXd& X,
                                            const Eigen::MatrixXd& Y,
                                            const Eigen::MatrixXd& A,
                                            const size_t TARGET_N_CLUSTER) const
{
    assert(X.rows()==Y.rows());
    assert(X.cols()==N_FEATURES);
    assert(X.rows()>=TARGET_N_CLUSTER);
    std::cout << "TARGET_N_CLUSTER=" << TARGET_N_CLUSTER << std::endl;
    
    std::vector<Eigen::VectorXd> centers;
    
    //choose initial centers
    std::vector<Eigen::VectorXd> lefts(X.rows());
    for (auto i=0; i < X.rows(); i++) { lefts.at(i) = X.row(i); }
    
    std::mt19937 mt;
    for (size_t k=0; k < TARGET_N_CLUSTER; k++) {
        std::deque<double> dists(1, 0.0);
        for (auto x : lefts) {
            double min_d = DBL_MAX;
            for (auto c : centers) {
                min_d = std::min(min_d, gaussianKernel_.sqsum(x.data(), c.data()));
            }
            dists.push_back(dists.back() + min_d);
        }
        dists.pop_front();
        assert(dists.size() == lefts.size());
        std::uniform_real_distribution<> rand(0.0, dists.back());
        const double oracle = rand(mt);
        const auto it_chosen = std::lower_bound(dists.begin(), dists.end(), oracle);
        const size_t idx_chosen = std::distance(dists.begin(), it_chosen);
        centers.push_back(lefts.at(idx_chosen));
        lefts.erase(lefts.begin() + idx_chosen);
        assert(centers.size() + lefts.size() == X.rows());
    }
    assert(centers.size() == TARGET_N_CLUSTER);
    
    //k-means
    std::vector<std::vector<size_t>> labels(TARGET_N_CLUSTER);
    const size_t MAX_ITERATION = 32;
    for (auto r=0; r < MAX_ITERATION; r++) {
        //clear previous labels
        for (auto i=0; i<labels.size(); i++) { labels.at(i).clear(); }
        
        //assign each sample to the nearest cluster
        double sqdist_sum = 0.0;
        for (auto i=0; i < X.rows(); i++) {
            std::vector<double> d;
            for (auto c : centers) {
                Eigen::VectorXd x = X.row(i);
                d.push_back(gaussianKernel_.sqsum(x.data(), c.data()));
            }
            const auto it_nearest = std::min_element(d.begin(), d.end());
            sqdist_sum += *it_nearest;
            const size_t iNearestCluster = std::distance(d.begin(), it_nearest);
            labels.at(iNearestCluster).push_back(i);
        }
        std::cout << "sqdist_sum[" << r << "]= " << sqdist_sum << std::endl;
        for (auto l : labels) { std::cout << l.size() << ","; }
        std::cout << std::endl;
        
        //calculate cluster centers
        for (auto k=0; k < labels.size(); k++) {
            const auto kth_label = labels.at(k);
            Eigen::MatrixXd Xnext(kth_label.size(), X.cols());
            for (auto i=0; i < kth_label.size(); i++) {
                Xnext.row(i) = X.row(kth_label.at(i));
            }
            centers.at(k) = Xnext.colwise().mean();
        }
    }
    
    CentroidBasedClusteringResult res;
    res.centers = centers;
    for (auto k=0; k < labels.size(); k++) {
        const auto kth_label = labels.at(k);
        Eigen::MatrixXd Xnext(kth_label.size(), X.cols());
        Eigen::MatrixXd Ynext(kth_label.size(), Y.cols());
        Eigen::MatrixXd Anext(kth_label.size(), A.cols());
        for (auto i=0; i < kth_label.size(); i++) {
            Xnext.row(i) = X.row(kth_label.at(i));
            Ynext.row(i) = Y.row(kth_label.at(i));
            Anext.row(i) = A.row(kth_label.at(i));
        }
        res.XC.push_back(Xnext);
        res.YC.push_back(Ynext);
        res.AC.push_back(Anext);
    }
    //need to remove empty clusters?
    
    return res;
}


loc::GaussianProcessLight::CentroidBasedClusteringResult
loc::GaussianProcessLight::clusteringGreedyWithMagicThreshold(const Eigen::MatrixXd& X,
                                                              const Eigen::MatrixXd& Y,
                                                              const double MAGIC_W_THRESHOLD) const
{
    assert(X.rows()==Y.rows());
    assert(X.cols()==N_FEATURES);
    std::cout << "MAGIC_W_THRESHOLD=" << MAGIC_W_THRESHOLD << std::endl;
    CentroidBasedClusteringResult res;
    for (int is=0; is < X.rows(); is++) {
        double x[N_FEATURES];
        double c[N_FEATURES];
        std::vector<double> w;
        for (auto c_k : res.centers) {
            for (int j=0; j < N_FEATURES; j++) {
                x[j] = X(is, j);
                c[j] = c_k(j);
            }
            w.push_back(gaussianKernel_.computeKernel(x, c));
        }
        //find nearest (=max weight) cluster index
        int i_max = -1;
        double max_w = 0.0;
        for (int k=0; k < w.size(); k++) {
            if (max_w < w[k]) {
                max_w = w[k];
                i_max = k;
            }
        }
        if (max_w > MAGIC_W_THRESHOLD) {
            //update i_max-th cluster
            Eigen::MatrixXd Xim(res.XC.at(i_max).rows() + 1, X.cols());
            Eigen::MatrixXd Yim(res.YC.at(i_max).rows() + 1, Y.cols());
            Xim << res.XC.at(i_max), X.row(is);
            Yim << res.YC.at(i_max), Y.row(is);
            res.XC.at(i_max) = Xim;
            res.YC.at(i_max) = Yim;
            res.centers.at(i_max) = Xim.colwise().mean();
            //                    std::cout << "updated " << i_max << "-th cluster: " << res.centers.at(i_max).transpose() << std::endl;
        } else {
            //create new cluster
            res.XC.push_back(X.row(is));
            res.YC.push_back(Y.row(is));
            res.centers.push_back(X.row(is));
            //                    std::cout << "new cluster: " << res.centers.at(res.nCluster()-1).transpose() << std::endl;
        }
    }
    
    return res;
}

loc::GaussianProcessLight::CentroidBasedClusteringResult
loc::GaussianProcessLight::clusteringGreedyWithMagicThreshold(const Eigen::MatrixXd& X,
                                                              const Eigen::MatrixXd& Y,
                                                              const Eigen::MatrixXd& A,
                                                              const double MAGIC_W_THRESHOLD) const
{
    assert(X.rows()==Y.rows());
    assert(X.cols()==N_FEATURES);
    std::cout << "MAGIC_W_THRESHOLD=" << MAGIC_W_THRESHOLD << std::endl;
    CentroidBasedClusteringResult res;
    for (int is=0; is < X.rows(); is++) {
        double x[N_FEATURES];
        double c[N_FEATURES];
        std::vector<double> w;
        for (auto c_k : res.centers) {
            for (int j=0; j < N_FEATURES; j++) {
                x[j] = X(is, j);
                c[j] = c_k(j);
            }
            w.push_back(gaussianKernel_.computeKernel(x, c));
        }
        //find nearest (=max weight) cluster index
        int i_max = -1;
        double max_w = 0.0;
        for (int k=0; k < w.size(); k++) {
            if (max_w < w[k]) {
                max_w = w[k];
                i_max = k;
            }
        }
        if (max_w > MAGIC_W_THRESHOLD) {
            //update i_max-th cluster
            Eigen::MatrixXd Xim(res.XC.at(i_max).rows() + 1, X.cols());
            Eigen::MatrixXd Yim(res.YC.at(i_max).rows() + 1, Y.cols());
            Eigen::MatrixXd Aim(res.AC.at(i_max).rows() + 1, A.cols());
            Xim << res.XC.at(i_max), X.row(is);
            Yim << res.YC.at(i_max), Y.row(is);
            Aim << res.AC.at(i_max), A.row(is);
            res.XC.at(i_max) = Xim;
            res.YC.at(i_max) = Yim;
            res.AC.at(i_max) = Aim;
            res.centers.at(i_max) = Xim.colwise().mean();
            //                    std::cout << "updated " << i_max << "-th cluster: " << res.centers.at(i_max).transpose() << std::endl;
        } else {
            //create new cluster
            res.XC.push_back(X.row(is));
            res.YC.push_back(Y.row(is));
            res.AC.push_back(A.row(is));
            res.centers.push_back(X.row(is));
            //                    std::cout << "new cluster: " << res.centers.at(res.nCluster()-1).transpose() << std::endl;
        }
    }
    
    return res;
}

/**
 * Improve prediction accuracy near cluster borders by permitting overlaps among clusters.
 * For each sample point in X, examine its weights against k-nearest centers of clusters and
 * for the nearest, add it to the cluster (=> any sample belongs to at least one cluster),
 * for the 2nd to k-nearests, add it to the cluster only if the weight exceeds OVERLAP_SCALE * (the weightest).
 */

void
loc::GaussianProcessLight::improveWithOverlap(CentroidBasedClusteringResult& cr,
                                              const double OVERLAP_SCALE,
                                              const Eigen::MatrixXd& X,
                                              const Eigen::MatrixXd& Y) const
{
    std::cout << "improve clusters with overlaps" << std::endl;
    std::cout << "OVERLAP_SCALE=" << OVERLAP_SCALE << std::endl;
    const size_t k = 3;
    const size_t n = cr.nCluster();
    std::vector<Eigen::VectorXd> empty;
    std::vector<std::vector<Eigen::VectorXd>> Xbuf(n, empty);
    std::vector<std::vector<Eigen::VectorXd>> Ybuf(n, empty);
    
    for (auto is=0; is < X.rows(); is++) {
        Eigen::VectorXd x = X.row(is);
        std::vector<double> weights(n);
        for (int i=0; i < n; ++i) {
            weights[i] = gaussianKernel_.computeKernel(x.data(), cr.centers.at(i).data());
        }
        std::vector<size_t> nearests = top_k(weights, std::min(k, n));
        
        Xbuf.at(nearests[0]).push_back(X.row(is));
        Ybuf.at(nearests[0]).push_back(Y.row(is));
        for (auto i=1; i < nearests.size(); i++) {
            if (weights[nearests[i]] > OVERLAP_SCALE * weights[nearests[0]]) {
                Xbuf.at(nearests[i]).push_back(X.row(is));
                Ybuf.at(nearests[i]).push_back(Y.row(is));
            }
        }
    }
    
    //remove empty rows
    const size_t n_bef = Xbuf.size();
    auto is_empty = [](const std::vector<Eigen::VectorXd>& v) { return v.empty(); };
    Xbuf.erase(remove_if(Xbuf.begin(), Xbuf.end(), is_empty), Xbuf.end());
    Ybuf.erase(remove_if(Ybuf.begin(), Ybuf.end(), is_empty), Ybuf.end());
    const size_t n_aft = Xbuf.size();
    std::cout << "n_cluster: " << n_bef << " -> " << n_aft << std::endl;
    
    //replace clusters to improved ones
    assert(Xbuf.size() == Ybuf.size());
    const size_t n_new = Xbuf.size();
    if (n_new != n) {
        cr.XC.resize(n_new);
        cr.YC.resize(n_new);
        cr.centers.resize(n_new);
    }
    for (auto i=0; i < n_new; i++) {
        assert(Xbuf.at(i).size() == Ybuf.at(i).size());
        const size_t n_ith = Xbuf.at(i).size();
        Eigen::MatrixXd Xnext(n_ith, X.cols());
        Eigen::MatrixXd Ynext(n_ith, Y.cols());
        for (auto j=0; j < n_ith; j++) {
            Xnext.row(j) = Xbuf[i][j];
            Ynext.row(j) = Ybuf[i][j];
        }
        cr.XC.at(i) = Xnext;
        cr.YC.at(i) = Ynext;
        cr.centers.at(i) = Xnext.colwise().mean();
    }
}

void
loc::GaussianProcessLight::improveWithOverlap(CentroidBasedClusteringResult& cr,
                                              const double OVERLAP_SCALE,
                                              const Eigen::MatrixXd& X,
                                              const Eigen::MatrixXd& Y,
                                              const Eigen::MatrixXd& A) const
{
    std::cout << "improve clusters with overlaps" << std::endl;
    std::cout << "OVERLAP_SCALE=" << OVERLAP_SCALE << std::endl;
    const size_t k = 3;
    const size_t n = cr.nCluster();
    std::vector<Eigen::VectorXd> empty;
    std::vector<std::vector<Eigen::VectorXd>> Xbuf(n, empty);
    std::vector<std::vector<Eigen::VectorXd>> Ybuf(n, empty);
    std::vector<std::vector<Eigen::VectorXd>> Abuf(n, empty);
    
    for (auto is=0; is < X.rows(); is++) {
        Eigen::VectorXd x = X.row(is);
        std::vector<double> weights(n);
        for (int i=0; i < n; ++i) {
            weights[i] = gaussianKernel_.computeKernel(x.data(), cr.centers.at(i).data());
        }
        std::vector<size_t> nearests = top_k(weights, std::min(k, n));
        
        Xbuf.at(nearests[0]).push_back(X.row(is));
        Ybuf.at(nearests[0]).push_back(Y.row(is));
        Abuf.at(nearests[0]).push_back(A.row(is));
        for (auto i=1; i < nearests.size(); i++) {
            if (weights[nearests[i]] > OVERLAP_SCALE * weights[nearests[0]]) {
                Xbuf.at(nearests[i]).push_back(X.row(is));
                Ybuf.at(nearests[i]).push_back(Y.row(is));
                Abuf.at(nearests[i]).push_back(A.row(is));
            }
        }
    }
    
    //remove empty rows
    const size_t n_bef = Xbuf.size();
    auto is_empty = [](const std::vector<Eigen::VectorXd>& v) { return v.empty(); };
    Xbuf.erase(remove_if(Xbuf.begin(), Xbuf.end(), is_empty), Xbuf.end());
    Ybuf.erase(remove_if(Ybuf.begin(), Ybuf.end(), is_empty), Ybuf.end());
    Abuf.erase(remove_if(Abuf.begin(), Abuf.end(), is_empty), Abuf.end());
    const size_t n_aft = Xbuf.size();
    std::cout << "n_cluster: " << n_bef << " -> " << n_aft << std::endl;
    
    //replace clusters to improved ones
    assert(Xbuf.size() == Ybuf.size());
    const size_t n_new = Xbuf.size();
    if (n_new != n) {
        cr.XC.resize(n_new);
        cr.YC.resize(n_new);
        cr.AC.resize(n_new);
        cr.centers.resize(n_new);
    }
    for (auto i=0; i < n_new; i++) {
        assert(Xbuf.at(i).size() == Ybuf.at(i).size());
        const size_t n_ith = Xbuf.at(i).size();
        Eigen::MatrixXd Xnext(n_ith, X.cols());
        Eigen::MatrixXd Ynext(n_ith, Y.cols());
        Eigen::MatrixXd Anext(n_ith, A.cols());
        for (auto j=0; j < n_ith; j++) {
            Xnext.row(j) = Xbuf[i][j];
            Ynext.row(j) = Ybuf[i][j];
            Anext.row(j) = Abuf[i][j];
        }
        cr.XC.at(i) = Xnext;
        cr.YC.at(i) = Ynext;
        cr.AC.at(i) = Anext;
        cr.centers.at(i) = Xnext.colwise().mean();
    }
}



//};
///**
// * NO GOOD PERFORMANCE! Aggregative hierarchical clustering.
// */
//loc::GaussianProcessLight::CentroidBasedClusteringResult
//loc::GaussianProcessLight::aggregativeClustering(const Eigen::MatrixXd& X,
//                                                 const Eigen::MatrixXd& Y,
//                                                 const size_t TARGET_N_CLUSTER) const
//{
//    assert(X.rows()==Y.rows());
//    assert(X.cols()==N_FEATURES);
//    std::cout << "TARGET_N_CLUSTER=" << TARGET_N_CLUSTER << std::endl;
//
//    std::vector<CentroidBasedCluster> clusters;
//    for (auto is=0; is < X.rows(); is++) {
//        clusters.emplace_back(is, X);
//    }
//
//    while (clusters.size() > TARGET_N_CLUSTER) {
//        //find min distance (= sqsum) pair
//        //TODO O(n^3) -> O(n^2)
//        double min_d = DBL_MAX;
//        std::pair<size_t,size_t> min_pair;
//        for (auto i=0; i < clusters.size(); i++) {
//            for (auto j=0; j < i; j++) {
//                double x[N_FEATURES];
//                double y[N_FEATURES];
//                for (int k=0; k < N_FEATURES; k++) {
//                    x[k] = clusters.at(i).center(k);
//                    y[k] = clusters.at(j).center(k);
//                    double d = gaussianKernel_.sqsum(x, y);
//                    if (d < min_d) {
//                        min_d = d;
//                        min_pair = std::make_pair(i, j);
//                    }
//                }
//            }
//        }
//
//        //merge the pair
//        clusters.at(min_pair.second).merge(clusters.at(min_pair.first));
//        clusters.erase(clusters.begin() + min_pair.first);
//    }
//    assert(clusters.size() == TARGET_N_CLUSTER);
//
//    CentroidBasedClusteringResult res;
//    for (auto c : clusters) {
//        Eigen::MatrixXd Xim(c.labels.size(), X.cols());
//        Eigen::MatrixXd Yim(c.labels.size(), Y.cols());
//        for (size_t i=0; i < c.labels.size(); i++) {
//            Xim.row(i) = X.row(c.labels.at(i));
//            Yim.row(i) = Y.row(c.labels.at(i));
//        }
//        res.XC.push_back(Xim);
//        res.YC.push_back(Yim);
//        res.centers.push_back(c.center);
//    }
//
//    return res;
//}
//
