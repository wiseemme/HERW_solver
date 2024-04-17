//Standard Libraries
#include <vector>

//External Packages
#include <sophus/se3.hpp>

class robotWorldDataSim{
    int num_meas;
    double scale = 1.0;
    Sophus::SE3d X, Y;
    std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>> data;
    void set_X();
    void set_Y();
    std::default_random_engine generator;

    public:
    //Constructors
    robotWorldDataSim(int n);
    
    //Data Generator
    void set_scale(double val);
    void generate_data();
    void generate_data_known_A();
    void generate_another_camera();

    //Data Collectors
    std::vector<std::pair<Sophus::SE3d, Sophus::SE3d>> get_data();
    
    //Check Results
    Sophus::SE3d get_X();
    Sophus::SE3d get_Y();
};
