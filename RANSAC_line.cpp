#include<iostream>
#include <vector>
#include <ctime>
#include<cmath>

template <typename T>
using Vec = std::vector<T>;

template <typename T1, typename T2>
using Pair = std::pair<T1, T2>;


class LineModel{
    public:
        double m = 0;
        double b = 0;

        // Default Constructor
        LineModel() = default;

        // Defined Constructor
        LineModel(const Pair<double, double> &p1, const Pair<double, double> &p2){
            if(p2.first - p1.first != 0) m = (p2.second - p1.second) / (p2.first - p1.first);
            else m = 1e10;
            b = p1.second - m * p1.first; }
        
        double computeError(const Pair<double, double> &pt) const {
            double y_estimated = m * pt.first + b;
            return std::abs(y_estimated - pt.second); }
};

class RANSAC{
    private:
        Vec<Pair<double, double>> data;
        double tolerance;
        int max_iterations;
        int threshold;

        LineModel FitLeastSquares(const Vec<Pair<double, double>> &points){
            double sumX = 0, sumY = 0, sumX2 = 0, sumXY = 0;
            int n = points.size();
            for (const auto& pt : points){
                sumX += pt.first;
                sumY += pt.second;
                sumX2 += pt.first * pt.first;
                sumXY += pt.first * pt.second; }
            
            double denom = n * sumX2 - sumX * sumX;
            if (denom == 0)
                return LineModel();

            double m = (n * sumXY - sumX * sumY) / denom;
            double b = (sumY - m * sumX) / n;

            LineModel line;
            line.m = m, line.b = b;
            return line;
        }
    
    public: 
        RANSAC(Vec<Pair<double, double>> points, double tolerance, int max_iterations, int threshold) 
            : data(points), tolerance(tolerance), max_iterations(max_iterations), threshold(threshold) {
                std::srand(static_cast<unsigned int>(std::time(nullptr))); }

        LineModel run() {
            LineModel bestModel;
            int bestInLiers = 0;

            for (int i=0; i<max_iterations; i++){
                Pair<double, double> pt1 = data[rand() % data.size()]; 
                Pair<double, double> pt2 = data[rand() % data.size()];
                while(pt1.first == pt2.first && pt1.second == pt2.second) Pair<double, double> pt2 = data[rand() % data.size()]; 

            LineModel model(pt1, pt2);
            Vec<Pair<double, double>> consensus_set;

            for(const auto &pt : data) if(model.computeError(pt) < tolerance) consensus_set.push_back(pt);

            if (consensus_set.size() > bestInLiers) { 
                bestInLiers = consensus_set.size();  
                bestModel = FitLeastSquares(consensus_set); }

            if (bestInLiers >= threshold) break;

            }

            return bestModel;
        }

};


int main(){

    Vec<Pair<double, double>> points = { 
        {0, 1.2}, {1, 3.1}, {2, 5.0}, {3, 6.8}, {4, 9.2},
        {5, 10.9}, {6, 13.0}, {7, 15.1}, {8, 16.8}, {9, 19.2},

        {1, 10.0}, {2, -3.5}, {3, 20.0}, {4, 1.0}, {6, 25.0},
        {7, -5.0}, {8, 30.0}, {10, -10.0}, {11, 35.0}, {12, 0.0}};

    RANSAC ransac(points, 0.5, 100, 10);
    LineModel best = ransac.run();

    std::cout << "Best line: y = " << best.m << "x + " << best.b << "\n";
    return 0;
}
