#include "wang_solver.hpp"

wangSolver::wangSolver(int N, int M){
    //Solver Constructor
    num_X = N;
    num_Y = M;
}

void wangSolver::add_datapoint(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index){
    append_rot_ave_cost_matrix(data, X_index, Y_index);
    append_trans_cost_matrix(data, X_index, Y_index);
    append_trans_offset_vector(data);
    append_trans_offset_matrix(data, Y_index);
}

void wangSolver::append_rot_ave_cost_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index){
    Eigen::SparseMatrix<double> temp_matrix(9, 9*(num_X+num_Y));
    temp_matrix.reserve(90);
    
    //Add Empty Columns
    for(int i=0; i<9*Y_index; i++){
        temp_matrix.startVec(i);
    }
    
    //Identity
    for(int i=0; i<9; i++){
        temp_matrix.startVec(i + 9 * Y_index);
        temp_matrix.insertBack(i, i + 9 * Y_index) = -1.0;
    }

    //Add Empty Columns
    for(int i=9*(Y_index + 1); i<9*(num_Y + X_index); i++){
        temp_matrix.startVec(i);
    }

    //Kronecker Product
    for(int i=0; i<9; i++){
        temp_matrix.startVec(9 * (num_Y + X_index) + i);
        for(int j=0; j<9; j++){
            temp_matrix.insertBack(j, 9 * (num_Y + X_index) + i) = *(data.second.rotationMatrix().data() + j/3 + 3*(i/3)) * *(data.first.rotationMatrix().data() + j % 3 + 3*(i%3));
        }
    }
    temp_matrix.finalize();
    //std::cout << Eigen::MatrixXd(temp_matrix) << std::endl;
    rot_ave_cost_matrix.push_back(temp_matrix);
}
    
void wangSolver::append_trans_cost_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int X_index, int Y_index){
    Eigen::SparseMatrix<double> temp_matrix(3, 3*(num_X+num_Y));
    temp_matrix.reserve(12);

    //Add Empty Columns
    for(int i=0; i<3*Y_index; i++){
        temp_matrix.startVec(i);
    }
    
    //Identity
    for(int i=0; i<3; i++){
        temp_matrix.startVec(i + 3 * Y_index);
        temp_matrix.insertBack(i, i + 3 * Y_index) = 1.0;
    }

    //Add Empty Columns
    for(int i=3*(Y_index + 1); i<3*(num_Y + X_index); i++){
        temp_matrix.startVec(i);
    }

    //Subtract Rotation Matrix
    for(int i=0; i<3; i++){
        temp_matrix.startVec(3 * (num_Y + X_index) + i);
        for(int j=0; j<3; j++){
            temp_matrix.insertBack(j, 3 * (num_Y + X_index) + i) = -1.0* *(data.first.rotationMatrix().data() + j + 3*i);
        }
    }
    temp_matrix.finalize();
    trans_cost_matrix.push_back(temp_matrix);
}

void wangSolver::append_trans_offset_vector(std::pair<Sophus::SE3d, Sophus::SE3d> data){
    trans_offset_vector.push_back(data.first.translation());
}

void wangSolver::append_trans_offset_matrix(std::pair<Sophus::SE3d, Sophus::SE3d> data, int Y_index){
    Eigen::SparseMatrix<double> temp_matrix(3, 9*(num_Y));
    temp_matrix.reserve(9);

    //Add Empty Columns
    for(int i=0; i<9*Y_index; i++){
        temp_matrix.startVec(i);
    }

    for(int i=0; i<9; i++){
        temp_matrix.startVec(i + 9 * Y_index);
        temp_matrix.insertBack(i%3, i + 9 * Y_index) = -1.0* data.second.translation().coeff(i/3);
    }
    temp_matrix.finalize();
    trans_offset_matrix.push_back(temp_matrix);
}

void wangSolver::rotation_solve(){
    //Concatenate Sparse Matrices
    Eigen::SparseMatrix<double> cost_matrix(9*rot_ave_cost_matrix.size(), 9*(num_X+num_Y));
    cost_matrix.reserve(90*rot_ave_cost_matrix.size());
    for(int i=0; i<9*(num_X+num_Y); i++){
        cost_matrix.startVec(i);
        for(int j=0; j<rot_ave_cost_matrix.size(); j++){
            for(Eigen::SparseMatrix<double>::InnerIterator inner(rot_ave_cost_matrix[j], i); inner; ++inner){
                cost_matrix.insertBack(inner.row() + 9 *j, i) = inner.value();
            }
        }
    }
    //std::cout << cost_matrix.hasNaN() << std::endl;
    Eigen::MatrixXd sym_cost_matrix = Eigen::MatrixXd((cost_matrix.transpose() * cost_matrix).pruned(1)); //prunes entries where |entry|< 1e-12
    sym_cost_matrix = (sym_cost_matrix + sym_cost_matrix.transpose())/2.0;
    //std::cout << sym_cost_matrix << std::endl;

    //Extract Rotation Solution
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(sym_cost_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd solution(svd.matrixU().col(9*(num_Y+num_X)-1)); //Last column is the smallest eigenvector
    //std::cout << svd.singularValues() << std::endl;
    Eigen::Matrix3d rotmatrix;

    //Save solved rotations 
    for(int i=0; i<(num_Y + num_X); i++){
        rotmatrix = Eigen::Matrix3d(solution.data() + 9*i);
        rotmatrix = ((rotmatrix.determinant() > 0) - (rotmatrix.determinant() < 0))/std::cbrt(((rotmatrix.determinant() > 0) - (rotmatrix.determinant() < 0))* rotmatrix.determinant())*rotmatrix;
        svd = Eigen::JacobiSVD<Eigen::MatrixXd>(rotmatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        rotmatrix = svd.matrixU()*svd.matrixV().transpose();
        if(i<num_Y){
            Y_poses.push_back(Sophus::SE3d(Sophus::SO3d(rotmatrix), Eigen::Vector3d::Zero()));
        }else{
            X_poses.push_back(Sophus::SE3d(Sophus::SO3d(rotmatrix), Eigen::Vector3d::Zero()));
        }
    }
}

void wangSolver::translation_solve(){

    //Concatenate Vectors of Sparse Matrices and Vectors
    Eigen::SparseMatrix<double> cost_matrix(3*trans_cost_matrix.size(), 3*(num_X+num_Y));
    cost_matrix.reserve(12*rot_ave_cost_matrix.size());
    for(int i=0; i<3*(num_X+num_Y); i++){
        cost_matrix.startVec(i);
        for(int j=0; j<trans_cost_matrix.size(); j++){
            for(Eigen::SparseMatrix<double>::InnerIterator inner(trans_cost_matrix[j], i); inner; ++inner){
                cost_matrix.insertBack(inner.row() + 3 *j, i) = inner.value();
            }
        }
    }
    cost_matrix.finalize();
    Eigen::VectorXd vec_joined(3*trans_offset_vector.size());
    for(int i=0; i<trans_offset_vector.size(); i++){
        vec_joined.block(3*i, 0, 3, 1) = trans_offset_vector[i];
    } 
    Eigen::SparseMatrix<double> offset_matrix(3*trans_cost_matrix.size(), 9*num_Y);
    offset_matrix.reserve(9*trans_offset_matrix.size());
    for(int i=0; i<9*(num_Y); i++){
        offset_matrix.startVec(i);
        for(int j=0; j<trans_offset_matrix.size(); j++){
            for(Eigen::SparseMatrix<double>::InnerIterator inner(trans_offset_matrix[j], i); inner; ++inner){
                offset_matrix.insertBack(inner.row() + 3 *j, i) = inner.value();
            }
        }
    }
    offset_matrix.finalize();
    Eigen::VectorXd rotation_vector(9*num_Y);
    for(int i=0; i<Y_poses.size(); i++){
        rotation_vector.block(9*i, 0, 9, 1) = Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(Y_poses[i].rotationMatrix().data(), 9));
    }
    vec_joined = vec_joined + offset_matrix*rotation_vector;

    //Multiply by Transpose(A)
    Eigen::SparseMatrix<double> sym_cost_matrix = (cost_matrix.transpose() * cost_matrix).pruned(1); //prunes entries where |entry|< 1e-12
    sym_cost_matrix = (sym_cost_matrix + Eigen::SparseMatrix<double>(sym_cost_matrix.transpose()))/2.0;
    vec_joined = cost_matrix.transpose() *vec_joined;

    //Solve
    Eigen::VectorXd translations;
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(sym_cost_matrix);
    translations = solver.solve(vec_joined);
    for(int i=0; i<(num_X + num_Y); i++){
        if (i<num_Y){
            Y_poses[i].translation() = translations.block(3*i, 0, 3, 1);
        } else{
            X_poses[i-num_Y].translation() = translations.block(3*i, 0, 3, 1);
        }
    }
}