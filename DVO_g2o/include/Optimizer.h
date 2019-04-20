/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/linear_solver_eigen.h"
#include "g2o/types/types_six_dof_expmap.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/linear_solver_dense.h"
#include "g2o/types/types_seven_dof_expmap.h"
#include<Eigen/StdVector>
#include "Converter.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class Optimizer
{
public:
    Optimizer();

    void add_Vertex(Eigen::Matrix4d T, int idx, bool fix);
    void add_Edge(int id_1, int id_2, Eigen::Matrix4d T21, double invSigma);
    //void add_Edge_Left(int id_1, int id_2, Eigen::Matrix3d R21, Eigen::Vector3d t21, double invSigma);
    void optimize_graph(int nIterations);
    Eigen::Matrix4d get_Pose_global(int idx);
    double get_error();
    void clean_graph();
    void odom_only(int start_idx, int end_idx);
   
private:
    g2o::SparseOptimizer optimizer;

};

#endif // OPTIMIZER_H
