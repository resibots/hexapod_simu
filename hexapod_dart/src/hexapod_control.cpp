#include <hexapod_dart/hexapod_control.hpp>

using namespace hexapod_dart;

HexapodControl::HexapodControl(const std::vector<double>& ctrl, robot_t robot) : _controller(ctrl, robot->broken_legs()), _robot(robot)
{
    _target_positions = _robot->skeleton()->getPositions();

    size_t dof = _robot->skeleton()->getNumDofs();

    _p = Eigen::VectorXd::Zero(dof);

    // first 6 DOF are 6d position - we don't want to put P values there
    for (size_t i = 0; i < 6; ++i) {
        _p(i) = 0.0;
    }

    for (size_t i = 6; i < dof; ++i) {
        _p(i) = 1.0;
    }
}

void HexapodControl::set_parameters(const std::vector<double>& ctrl)
{
    _controller.set_parameters(ctrl);
}

std::vector<double> HexapodControl::parameters()
{
    return _controller.parameters();
}

HexapodControl::robot_t HexapodControl::robot()
{
    return _robot;
}

void HexapodControl::update(double t)
{
    auto angles = _controller.pos(t);
    for (size_t i = 0; i < angles.size(); i++)
        _target_positions(i + 6) = angles[i];

    set_commands();
}

void HexapodControl::set_commands()
{
    if (_robot == nullptr)
        return;

    Eigen::VectorXd q = _robot->skeleton()->getPositions();
    Eigen::VectorXd q_err = _target_positions - q;

    double gain = 1.0 / (DART_PI * _robot->skeleton()->getTimeStep());
    Eigen::VectorXd vel = q_err * gain;
    vel = vel.cwiseProduct(_p);

    _robot->skeleton()->setCommands(vel);
}
