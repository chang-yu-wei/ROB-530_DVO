#include "Optimizer.h"

Optimizer::Optimizer()
{
     g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
}

void Optimizer::add_Vertex(Eigen::Matrix4d T, int idx, bool fix)
{
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    Eigen::Matrix4d T_inv = T.inverse();
    vSE3->setEstimate(g2o::SE3Quat(T_inv.block<3,3>(0,0),T_inv.block<3,1>(0,3))); // from world to camera T_wc
    vSE3->setId(idx);
    vSE3->setFixed(fix);
    optimizer.addVertex(vSE3);
}


void Optimizer::add_Edge(int id_1, int id_2, Eigen::Matrix4d T21, double invSigma)
{
    //
    g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();

    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id_1)));
    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id_2)));
    e->setMeasurement(g2o::SE3Quat(T21.block<3,3>(0,0),T21.block<3,1>(0,3)));
    e->setInformation(Matrix6d::Identity()*invSigma);

    // if(bRobust)
    // {
    //     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //     e->setRobustKernel(rk);
    //     rk->setDelta(thHuber2D);
    // }

    optimizer.addEdge(e);
}


void Optimizer::optimize_graph(int nIterations)
{
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    optimizer.optimize(nIterations);
    

}

Eigen::Matrix4d Optimizer::get_Pose_global(int idx)
{
    g2o::VertexSE3Expmap* vi = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(idx));
    g2o::SE3Quat T_iG = vi->estimate();
    g2o::SE3Quat T_Gi = T_iG.inverse();
    return T_Gi.to_homogeneous_matrix();
}

double Optimizer::get_error()
{
    return optimizer.chi2();
}

void Optimizer::clean_graph()
{
    optimizer.clear();
}

