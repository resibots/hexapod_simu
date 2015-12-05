#include <hexapod_control.hpp>

HexapodControl::HexapodControl(const std::vector<double>& ctrl, robot_t robot) : _controller(ctrl, robot->broken_legs())
{
    _target_positions = _robot->skeleton()->getPositions();

    size_t dof = _robot->skeleton()->getNumDofs();

    _forces = Eigen::VectorXd::Zero(dof);

    _kp = Eigen::MatrixXd::Identity(dof, dof);
    _kd = Eigen::MatrixXd::Identity(dof, dof);

    // first 6 DOF are 6d position - we don't want to put PD values there
    for (size_t i = 0; i < 6; ++i) {
        _kp(i, i) = 0.0;
        _kd(i, i) = 0.0;
    }

    // TO-DO: check PD gains
    for (size_t i = 6; i < dof; ++i) {
        _kp(i, i) = 100.0;
        _kd(i, i) = 15.0;
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

    set_forces();
}

void HexapodControl::set_forces()
{
    if (_robot == nullptr)
        return;
    // TO-DO: Try different controllers
    Eigen::VectorXd q = _robot->skeleton()->getPositions();
    Eigen::VectorXd dq = _robot->skeleton()->getVelocities();

    Eigen::MatrixXd invM = (_robot->skeleton()->getMassMatrix()
                               + _kd * _robot->skeleton()->getTimeStep()).inverse();
    Eigen::VectorXd p = -_kp * (q + dq * _robot->skeleton()->getTimeStep() - _target_positions);
    Eigen::VectorXd d = -_kd * dq;
    Eigen::VectorXd qddot = invM * (-_robot->skeleton()->getCoriolisAndGravityForces()
                                       + p + d + _robot->skeleton()->getConstraintForces());

    _forces = p + d - _kd * qddot * _robot->skeleton()->getTimeStep();

    _robot->skeleton()->setForces(_forces);
}