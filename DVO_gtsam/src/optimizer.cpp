#include "optimizer.h"



DVO_optimizer::DVO_optimizer()
{
    // initialize odom Cov
    Vector6 Sigma_odom;
    Sigma_odom << 1.0, 1.0, 1.0, 0.5, 0.5, 0.5;
    odomModel = noiseModel::Diagonal::Sigmas(Sigma_odom);

    Vector6 Sigma_loop;
    Sigma_loop << 2.0, 2.0, 2.0, 1, 1, 1;
    loopModel = noiseModel::Diagonal::Sigmas(Sigma_loop);
}

void DVO_optimizer::addVertex(int idx, Rot3 rot, Point3 t)
{
    // fix first node
    if(num_node == 0)
    {
        Vector6 Sigma;
        Sigma << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Sigma);
        graph.add(PriorFactor<Pose3>(Symbol('x', idx), Pose3(rot, t), priorModel));
    }
    initials.insert(Symbol('x',idx), Pose3(rot,t));
    num_node++;
}

void DVO_optimizer::addVertex(int idx, Eigen::Matrix4d T)
{
    // fix first node
    if(num_node == 0)
    {
        Vector6 Sigma;
        Sigma << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        noiseModel::Diagonal::shared_ptr priorModel = noiseModel::Diagonal::Sigmas(Sigma);
        graph.add(PriorFactor<Pose3>(Symbol('x', idx), Pose3(T), priorModel));
    }
    initials.insert(Symbol('x',idx), Pose3(T));
    num_node++;
}

void DVO_optimizer::addOdom(int idx_from, int idx_to, Rot3 rot, Point3 t)
{
    graph.add(BetweenFactor<Pose3>(Symbol('x', idx_from), Symbol('x', idx_to), Pose3(rot, t), odomModel));
}

void DVO_optimizer::addOdom(int idx_from, int idx_to, Eigen::Matrix4d T)
{
    graph.add(BetweenFactor<Pose3>(Symbol('x', idx_from), Symbol('x', idx_to), Pose3(T), odomModel));
}

void DVO_optimizer::addLC(int idx_from, int idx_to, Eigen::Matrix4d T)
{
    graph.add(BetweenFactor<Pose3>(Symbol('x', idx_from), Symbol('x', idx_to), Pose3(T), loopModel));
}

void DVO_optimizer::optimize()
{
    //sinitials.print("\nInitial Values:\n"); 
    GaussNewtonParams parameters;
  
    // print per iteration
    parameters.setVerbosity("ERROR");
    
    // optimize!
    GaussNewtonOptimizer optimizer(graph, initials, parameters);
    results = optimizer.optimize();
    
    // print final values
    //results.print("Final Result:\n");
}