#include "robot_world_sim.hpp"
#include "wang_solver.hpp"
#include "time.h"
#include <iostream>
#include <fstream>

const int num_poses = 10;

int main(){
    //Initialize Random Seed
    std::srand(time(NULL));

    // Constructing the simulated robot poses, called "sim" 
    std::cout << "Starting up by generating the simulated data..." << std::endl;
    robotWorldDataSim sim(num_poses);
    
    // Getting the simulated data and stored inside data 
    std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>> data = sim.get_data();
    Sophus::SE3d X_gt = sim.get_X();
    Sophus::SE3d Y_gt = sim.get_Y();
    
    // Create Wang Solver and Solve
    wangSolver wSolver(1, 1);
    for(std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>>::iterator ditr=data.begin(); ditr!=data.end(); ditr++){
        wSolver.add_datapoint(*ditr, 0, 0);
    }
    wSolver.solve();

    // Print Solution
    std::cout << "Ground Truth X:" << std::endl << sim.get_X().matrix() << std::endl;
    std::cout << "Estimated X:" << std::endl << wSolver.X_poses[0].matrix() << std::endl;
    std::cout << "Ground Truth Y:" << std::endl << sim.get_Y().matrix() << std::endl;
    std::cout << "Estimated Y:" << std::endl << wSolver.Y_poses[0].matrix() << std::endl;
    
    return 1;
}