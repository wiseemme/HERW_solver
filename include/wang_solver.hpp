// Standard Libraries
#include <vector>
#include <iostream>

// External Packages
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>


class wangSolver{
    public:

    //Cost term Storage
    std::vector<Eigen::SparseMatrix<double>> rot_ave_cost_matrix;
    std::vector<Eigen::SparseMatrix<double>> trans_cost_matrix;
    std::vector<Eigen::Vector3d> trans_offset_vector;
    std::vector<Eigen::SparseMatrix<double>> trans_offset_matrix;

    //Results Storage
    std::vector<Sophus::SE3d> X_poses;
    std::vector<Sophus::SE3d> Y_poses;

    //Variables
    int num_X;
    int num_Y;

    //Solvers
    void rotation_solve();
    void translation_solve();
    void solve(){rotation_solve();translation_solve();};

    //public:
    //Data defintion
    typedef std::pair<Sophus::SE3d, Sophus::SE3d> pose_pair;
    
    //Constructors
    wangSolver(int N, int M);

    //Data Functions
    void add_datapoint(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index);
    void append_rot_ave_cost_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index);
    void append_trans_cost_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index);
    void append_trans_offset_vector(std::pair<Sophus::SE3d, Sophus::SE3d> data);
    void append_trans_offset_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int Y_index);
    
    //Getters
    std::vector<Sophus::SE3d> get_X_poses();
    std::vector<Sophus::SE3d> get_Y_poses();
};