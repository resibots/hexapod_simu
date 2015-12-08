#include <iostream>
#include <hexapod_dart_simu.hpp>

int main()
{
    std::vector<int> brk = {};

    auto global_robot = std::make_shared<robot::Hexapod>("/home/kchatzil/Workspaces/DART/source/dart_test/pexod.urdf", brk);

    std::vector<double> ctrl;
    ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};

    HexapodDARTSimu simu(ctrl, global_robot);
    // for(int i=0;i<167;i++)
    //     simu.run(0.015, true);

    // ctrl = {1.0, 0.55, 0.45, 0.9, 0.6, 0.15, 0.95, 0.65, 0.15, 0.25, 0.85, 0.35, 1.0, 0.2, 0.6, 0.5, 0.05, 0.6, 1.0, 1.0, 0.4, 0.9, 0.75, 0.3, 1.0, 0.7, 0.55, 0.8, 0.25, 0.3, 1.0, 0.15, 0.3, 0.1, 0.4, 1.0};
    // // ctrl = {0.75, 1.0, 0.8, 0.8, 0.1, 0.95, 0.7, 0.2, 0.35, 0.85, 0.85, 0.95, 0.7, 1.0, 0.8, 0.7, 0.2, 1.0, 1.0, 0.7, 0.45, 0.15, 0.15, 0.75, 1.0, 0.15, 0.5, 0.1, 0.5, 0.85, 0.25, 0.1, 0.9, 0.2, 0.9, 0.9};
    // simu.controller().set_parameters(ctrl);
    simu.run(5);
    std::cout << simu.covered_distance() << " " << simu.arrival_angle() << std::endl;

    global_robot.reset();
    return 0;
}