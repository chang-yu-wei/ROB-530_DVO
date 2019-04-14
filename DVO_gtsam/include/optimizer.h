#ifndef OPTIMIZER_H
#define OPTIMIZER_H

// In planar cases we use Pose2 variables (x, y, theta) to represent the robot poses in SE(2)
#include <gtsam/geometry/Pose3.h>

// class for factor graph, a container of various factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// class for graph nodes values, a container of various geometric types
// here Values is used as a container of SE(2)
#include <gtsam/nonlinear/Values.h>

// symbol class is used to index varible in values
// e.g. pose varibles are generally indexed as 'x' + number, and landmarks as 'l' + numbers 
#include <gtsam/inference/Symbol.h>

// Factors used in this examples
// PriorFactor gives the prior distribution over a varible
// BetweenFactor gives odometry constraints
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// optimizer class, here we use Gauss-Newton
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the 
// (appoximated / linearized) marginal covariance of desired variables
#include <gtsam/nonlinear/Marginals.h>

#include <Eigen/Geometry>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace gtsam;

class DVO_optimizer
{
private:
    //main graph
    NonlinearFactorGraph graph;
    Values initials; //initial value for the vertex
    

    // useful value
    const Rot3 I_33 = Rot3(1,0,0,0,1,0,0,0,1);

    noiseModel::Diagonal::shared_ptr odomModel;
    noiseModel::Diagonal::shared_ptr loopModel;

    int num_node = 0;

public:
    Values results;
    
    DVO_optimizer();

    void addVertex(int idx, Rot3 rot, Point3 t);
    void addVertex(int idx, Eigen::Matrix4d T);
    void addOdom(int idx_from, int idx_to, Rot3 rot, Point3 t);
    void addOdom(int idx_from, int idx_to, Eigen::Matrix4d T );
    void addLC(int idx_from, int idx_to, Eigen::Matrix4d T);

    void optimize();

};
#endif 