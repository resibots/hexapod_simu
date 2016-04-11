#include <iostream>
#include <cstdlib>
#include <hexapod_dart/hexapod_dart_simu.hpp>

int main()
{
    std::vector<int> brk = {};

    auto global_robot = std::make_shared<hexapod_dart::Hexapod>(std::string(std::getenv("RESIBOTS_DIR")) + "/share/hexapod_models/URDF/pexod.urdf", brk);

    std::vector<double> ctrl;
    ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};
    // ctrl = {1, 0.85, 0.95, 1, 0.35, 0.2, 0.85, 0.05, 0.3, 0.95, 0.85, 0.4, 0.8, 0.55, 0.15, 0.25, 0.15, 0.85, 0.85, 0.85, 0.6, 0.5, 0.95, 0.05, 1, 0.25, 0.8, 0.75, 0.9, 0.3, 0.95, 0.3, 0.3, 0.65, 0.7, 0.75};
    // ctrl = {0.9, 1.0, 1.0, 0.75, 0.4, 0.95, 0.8, 0.9, 0.15, 0.2, 0.9, 0.15, 0.85, 0.3, 0.4, 0.8, 0.8, 0.7, 1.0, 0.9, 0.65, 0.1, 0.6, 0.4, 1.0, 0.75, 0.8, 0.8, 0.8, 0.75, 0.35, 0.35, 0.5, 0.8, 0.75, 0.85};
    // ctrl = {0.75, 0.9, 0.7, 0.75, 0.9, 0, 1, 0.1, 0.35, 0.2, 0.1, 0.15, 0.95, 0.25, 0.15, 0, 0.15, 0.15, 0.85, 0.95, 0.6, 0.9, 1, 0.2, 0.9, 0.85, 0.8, 0.25, 0.6, 0.95, 1, 0, 0.6, 0.3, 0.85, 0.75};
    // ctrl = {0.05, 0.8, 0.1, 0.9, 0.7, 0.35, 0.9, 1, 0.3, 0.7, 0.05, 0.15, 0.95, 0.75, 0.7, 0.95, 0.6, 0.6, 0.75, 0.7, 0.95, 0.6, 0.3, 0.35, 1, 0.4, 0.65, 0.55, 0.25, 0.6, 0.9, 0.75, 0.4, 1, 0.7, 0.85};

    using desc_t = boost::fusion::vector<hexapod_dart::descriptors::DutyCycle, hexapod_dart::descriptors::RotationTraj>;
    hexapod_dart::HexapodDARTSimu<hexapod_dart::desc<desc_t>> simu(ctrl, global_robot);
    // for(int i=0;i<167;i++)
    //     simu.run(0.015, true);

    // ctrl = {1.0, 0.55, 0.45, 0.9, 0.6, 0.15, 0.95, 0.65, 0.15, 0.25, 0.85, 0.35, 1.0, 0.2, 0.6, 0.5, 0.05, 0.6, 1.0, 1.0, 0.4, 0.9, 0.75, 0.3, 1.0, 0.7, 0.55, 0.8, 0.25, 0.3, 1.0, 0.15, 0.3, 0.1, 0.4, 1.0};
    // ctrl = {0.75, 1.0, 0.8, 0.8, 0.1, 0.95, 0.7, 0.2, 0.35, 0.85, 0.85, 0.95, 0.7, 1.0, 0.8, 0.7, 0.2, 1.0, 1.0, 0.7, 0.45, 0.15, 0.15, 0.75, 1.0, 0.15, 0.5, 0.1, 0.5, 0.85, 0.25, 0.1, 0.9, 0.2, 0.9, 0.9};
    // simu.controller().set_parameters(ctrl);
    simu.run(5);
    std::cout << simu.covered_distance() << " " << simu.arrival_angle() << std::endl;
    std::cout << simu.energy() << std::endl;
    std::vector<double> v;
    simu.get_descriptor<hexapod_dart::descriptors::DutyCycle>(v);
    for (size_t i = 0; i < v.size(); i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
    std::vector<double> vv;
    simu.get_descriptor<hexapod_dart::descriptors::RotationTraj>(vv);
    for (size_t i = 0; i < vv.size(); i++) {
        std::cout << vv[i] << " ";
    }
    std::cout << std::endl;

    global_robot.reset();
    return 0;
}
