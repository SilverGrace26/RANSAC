#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <numeric>   
#include <random>
#include <algorithm> 
#include <iterator>
#include <Eigen/Dense>

template <typename T>
using Vec = std::vector<T>;
using Point3d = Eigen::Vector3d;

class PlaneModel{
    public:
        double a = 0, b = 0, c = 0, d = 0;
        Point3d normal; 

        PlaneModel() = default;

        PlaneModel(const Point3d &pt1, const Point3d &pt2, const Point3d &pt3){
            Point3d V1 = pt2 - pt1;
            Point3d V2 = pt3 - pt1;

            Point3d cross_product = V1.cross(V2); 

            if (cross_product.norm() < 1e-9) { 
                a = b = c = d = 0; 
                return; 
            }

            normal = cross_product.normalized();

            a = normal.x();
            b = normal.y();
            c = normal.z();
            d = -normal.dot(pt1);
        }

        PlaneModel(const Point3d& normal_vec, const Point3d& centroid){
            this->normal = normal_vec.normalized(); 
            a = this->normal.x();
            b = this->normal.y();
            c = this->normal.z();
            d = -this->normal.dot(centroid);
        }
        
        double computeDistance(const Point3d &pt) const {
            if (a*a + b*b + c*c < 1e-18) return 1e10; 
            // Normal is already normalized, so denominator = 1
            return std::abs(a * pt.x() + b * pt.y() + c * pt.z() + d); 
        }
        
        bool isValid() const {
            return normal.norm() > 1e-9;
        }
};

class RANSAC{
    private:
        Vec<Point3d> data;
        double error_tolerance;
        int max_iterations;
        int min_consensus;
        std::mt19937 rng;

        PlaneModel fitModel(const Vec<Point3d>& consensus_set){
            if (consensus_set.size() < 3) return PlaneModel(); 

            // Finding the Centroid
            Point3d centroid = Point3d::Zero();
            for (const auto& pt : consensus_set) centroid += pt;
            centroid /= consensus_set.size();

            // Centering every row through a matrix
            Eigen::MatrixXd centered_data(consensus_set.size(), 3);
            for (size_t i = 0; i < consensus_set.size(); ++i) {
                centered_data.row(i) = (consensus_set[i] - centroid).transpose();
            }
            
            // Extracting normal vector through SVD
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered_data, Eigen::ComputeThinV);
            Point3d normal_vector = svd.matrixV().col(2);

            // Ensure consistent normal orientation
            if (normal_vector.dot(centroid) > 0) {
                normal_vector = -normal_vector;
            }

            return PlaneModel(normal_vector, centroid);
        }

        Vec<Point3d> getConsensusSet(const PlaneModel& model) const {
            Vec<Point3d> consensus_set;
            if (!model.isValid()) return consensus_set;
            
            for (const auto &pt : data) {
                if (model.computeDistance(pt) < error_tolerance) {
                    consensus_set.push_back(pt);
                }
            }
            return consensus_set;
        }

        bool areCollinear(const Point3d& p1, const Point3d& p2, const Point3d& p3) const {
            Point3d v1 = p2 - p1;
            Point3d v2 = p3 - p1;
            return v1.cross(v2).norm() < 1e-9;
        }

    public:
        RANSAC(Vec<Point3d> points, double error_tolerance, int max_iterations, int min_consensus) 
        : data(points), error_tolerance(error_tolerance), max_iterations(max_iterations), 
          min_consensus(min_consensus), rng(std::random_device{}()) {}
        
        PlaneModel run() {
            if (data.size() < 3) {
                std::cerr << "Insufficient data points for plane fitting." << std::endl;
                return PlaneModel();
            }

            int bestInliersCount = 0;
            Vec<Point3d> bestConsensusSet;
            int attempts_without_improvement = 0;
            const int max_attempts_without_improvement = max_iterations / 4;

            for (int i = 0; i < max_iterations; i++) {
                // Random Sampling with improved strategy
                Vec<int> indices(data.size());
                std::iota(indices.begin(), indices.end(), 0);
                std::shuffle(indices.begin(), indices.end(), rng);
                
                // Find three non-collinear points
                PlaneModel currentModel;
                bool found_valid_sample = false;
                
                for (int attempt = 0; attempt < std::min(10, (int)data.size() - 2); attempt++) {
                    if (indices.size() < 3) break;
                    
                    Point3d p1 = data[indices[attempt]];
                    Point3d p2 = data[indices[attempt + 1]];
                    Point3d p3 = data[indices[attempt + 2]];
                    
                    if (!areCollinear(p1, p2, p3)) {
                        currentModel = PlaneModel(p1, p2, p3);
                        if (currentModel.isValid()) {
                            found_valid_sample = true;
                            break;
                        }
                    }
                }
                
                if (!found_valid_sample) continue;

                // Get consensus set for current model 
                Vec<Point3d> currentConsensusSet = getConsensusSet(currentModel);

                // Only update if we found more inliers 
                if (static_cast<int>(currentConsensusSet.size()) > bestInliersCount) {
                    bestInliersCount = currentConsensusSet.size();
                    bestConsensusSet = currentConsensusSet;
                    attempts_without_improvement = 0;
                } else {
                    attempts_without_improvement++;
                }

                // Early termination conditions
                if (bestInliersCount >= min_consensus && 
                    (attempts_without_improvement > max_attempts_without_improvement)) {
                    break;
                }
            }

            // Final model fitting with best consensus set 
            if (!bestConsensusSet.empty() && bestConsensusSet.size() >= 3) {
                PlaneModel finalModel = fitModel(bestConsensusSet);
                if (finalModel.isValid()) {
                    std::cout << "RANSAC converged with " << bestInliersCount << " inliers out of " << data.size() << " points." << std::endl;
                    return finalModel;
                }
            }
            
            std::cerr << "RANSAC failed to find a valid consensus set." << std::endl;
            return PlaneModel();
        }

        // Method to evaluate model quality
        double evaluateModel(const PlaneModel& model) const {
            if (!model.isValid()) return 1e10;
            
            double total_error = 0.0;
            int inlier_count = 0;
            
            for (const auto& pt : data) {
                double dist = model.computeDistance(pt);
                if (dist < error_tolerance) {
                    total_error += dist;
                    inlier_count++;
                }
            }
            
            if (inlier_count == 0) return 1e10;
            return total_error / inlier_count; 
        }
};      

int main() {
    
    Vec<Point3d> points;

    // Inlier points (z = 2x + 0.5y + 1, or 2x + 0.5y - z + 1 = 0)
    points.push_back(Point3d(1.0, 1.0, 3.5)); 
    points.push_back(Point3d(2.0, 1.0, 5.5)); 
    points.push_back(Point3d(1.0, 2.0, 4.0)); 
    points.push_back(Point3d(3.0, 2.0, 8.0)); 
    points.push_back(Point3d(0.0, 0.0, 1.0)); 
    points.push_back(Point3d(2.5, 1.5, 7.25)); 
    points.push_back(Point3d(1.5, 0.5, 4.25));  
    points.push_back(Point3d(0.5, 1.5, 2.75));  

    // Outliers
    points.push_back(Point3d(10.0, 10.0, 10.0)); 
    points.push_back(Point3d(10.0, 20.0, 10.0)); 
    points.push_back(Point3d(5.0, 5.0, 100.0));    
    points.push_back(Point3d(-5.0, -5.0, -5.0));   
    points.push_back(Point3d(50.0, 1.0, 1.0));     
    points.push_back(Point3d(20.0, 20.0, 5.0));    
    points.push_back(Point3d(1.0, 1.0, -50.0));    
    points.push_back(Point3d(-10.0, 10.0, 10.0));

    double tolerance = 0.4; 
    int iterations = 2000;  
    int min_pts_for_consensus = 0.6 * points.size(); 

    RANSAC ransac_solver(points, tolerance, iterations, min_pts_for_consensus);
    PlaneModel best_fitted_plane = ransac_solver.run();

    std::cout << "\n--- RANSAC Results ---" << std::endl;
    if (best_fitted_plane.isValid()) { 
        std::cout << "Best fitted plane equation: "
                  << best_fitted_plane.a << "x + "
                  << best_fitted_plane.b << "y + "
                  << best_fitted_plane.c << "z + "
                  << best_fitted_plane.d << " = 0" << std::endl;
        std::cout << "Normal vector: (" << best_fitted_plane.a << ", " << best_fitted_plane.b << ", " << best_fitted_plane.c << ")" << std::endl;
        
        // Test multiple points
        Vec<Point3d> test_points = {
            Point3d(1.0, 1.0, 3.5),   
            Point3d(2.0, 1.0, 5.5),   
            Point3d(10.0, 10.0, 10.0) 
        };
        
        for (const auto& test_pt : test_points) {
            std::cout << "Distance from (" << test_pt.x() << "," << test_pt.y() << "," << test_pt.z() 
                      << ") to fitted plane: " << best_fitted_plane.computeDistance(test_pt) << std::endl;
        }
        
        // Evaluate model quality
        std::cout << "Average inlier error: " << ransac_solver.evaluateModel(best_fitted_plane) << std::endl;
        
        // Count inliers
        int inlier_count = 0;
        for (const auto& pt : points) {
            if (best_fitted_plane.computeDistance(pt) < tolerance) {
                inlier_count++;
            }
        }
        std::cout << "Total inliers: " << inlier_count << " out of " << points.size() << " points" << std::endl;
        
    } else {
        std::cout << "RANSAC failed to find a valid plane model." << std::endl;
    }

    return 0;
}