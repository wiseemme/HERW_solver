#include "robot_world_sim.hpp"
#include <iostream>

robotWorldDataSim::robotWorldDataSim(int n){
    num_meas = n;
    set_X();
    set_Y();
    generate_data();   
}

void robotWorldDataSim::set_X(){
    //Eigen::Matrix4d T = Eigen::Matrix4d::Random();
    X = Sophus::SE3d::sampleUniform(generator);
    X.translation() *= 0.1/X.translation().norm();
    // std::cout << "X matrix looks like: " << std::endl << X.matrix() << std::endl;
}

void robotWorldDataSim::set_Y(){
    //Eigen::Matrix4d T = Eigen::Matrix4d::Random();
    Y = Sophus::SE3d::sampleUniform(generator);
    // std::cout << "Y matrix looks like: " << std::endl << Y.matrix() << std::endl;
}

void robotWorldDataSim::generate_data(){
    data.clear();
    Sophus::SE3d B_temp, A_temp;
    Eigen::Vector3d trans_temp;
    Eigen::Vector3d fixed_vec;

    fixed_vec = 0.5*Eigen::Vector3d::Random();

    for (int i = 0; i<num_meas; i++){
        // B_temp = Sophus::SE3d::fitToSE3(Eigen::Matrix4d::Random())
        trans_temp = Eigen::Vector3d::Random();
        trans_temp = trans_temp/trans_temp.norm();
        B_temp = Sophus::SE3d(Sophus::SO3d::sampleUniform(generator), 0.1 * rand()/RAND_MAX* trans_temp + fixed_vec); //Failure Case 1 camera lies on a sphere
        //B_temp = Sophus::SE3d(Sophus::SO3d::sampleUniform(generator), 0.1 * Eigen::Vector3d::Random()); // Failure Case 2 camera does not move a lot
        //B_temp = Sophus::SE3d::sampleUniform(generator);
        // std::cout << "The randomly generated B matrix looks like: " << std::endl << B_temp.matrix() << std::endl;
        A_temp = Y * B_temp * X.inverse();
        // std::cout << "The randomly generated A matrix looks like: " << std::endl << A_temp.matrix() << std::endl;
        B_temp.translation() *= scale;  
        data.push_back(std::make_pair(A_temp, B_temp));
    }
}

void robotWorldDataSim::generate_data_known_A(){
    Sophus::SE3d B_temp;
    for(std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>::iterator dit=data.begin(); dit!=data.end(); dit++){
        B_temp = Y.inverse() * (dit->first) * X;
        B_temp.translation() *= scale;
        dit->second = B_temp;
    }
}

void robotWorldDataSim::generate_another_camera(){
    set_X();
    generate_data_known_A();
}

std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>> robotWorldDataSim::get_data(){
    return data;
}

Sophus::SE3d robotWorldDataSim::get_X(){
    return X;
}

Sophus::SE3d robotWorldDataSim::get_Y(){
    return Y;
}

void robotWorldDataSim::set_scale(double val){
    scale = val;
}