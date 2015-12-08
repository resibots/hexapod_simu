#ifndef HEXAPOD_DART_HEXAPOD_CONTROL
#define HEXAPOD_DART_HEXAPOD_CONTROL

#include <hexapod.hpp>
#include <hexapod_controller_simple.hpp>

class HexapodControl {
public:
    using robot_t = std::shared_ptr<robot::Hexapod>;

    HexapodControl() {}
    HexapodControl(const std::vector<double>& ctrl, robot_t robot);

    void set_parameters(const std::vector<double>& ctrl);
    std::vector<double> parameters();

    robot_t robot();

    void update(double t);

    void set_commands();

protected:
    HexapodControllerSimple _controller;
    robot_t _robot;

    Eigen::VectorXd _target_positions;
    // Eigen::MatrixXd _kp;
    // Eigen::MatrixXd _kd;
    Eigen::VectorXd _p;
    // Eigen::VectorXd _forces;
};

#endif