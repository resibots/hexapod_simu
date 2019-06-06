#ifndef HEXAPOD_DART_HEXAPOD_CPG_OPEN_LOOP_CONTROL
#define HEXAPOD_DART_HEXAPOD_CPG_OPEN_LOOP_CONTROL

#include <hexapod_controller/cpg_open_loop.hpp>
#include <hexapod_dart/hexapod.hpp>
#include <iostream>
namespace hexapod_dart {

    class HexapodCPGOpenLoopControl {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodCPGOpenLoopControl() {}
        HexapodCPGOpenLoopControl(const std::vector<double>& ctrl, robot_t robot)
        {

            _broken_legs = robot->broken_legs();

            //  _controller.set_parameters(ctrl, _broken_legs);

            _robot = robot;

            size_t dof = _robot->skeleton()->getNumDofs();

            _vel_cmd = Eigen::VectorXd::Zero(dof);

            _controller.computeTrajectory(5);

            _p = Eigen::VectorXd::Zero(dof);

            // first 6 DOF are 6d position - we don't want to put P values there
            for (size_t i = 0; i < 6; ++i) {
                _p(i) = 0.0;
            }

            for (size_t i = 6; i < dof; ++i) {
                _p(i) = 1.0;
            }
            std::cout << "end constructor" << std::endl;
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            //  _controller.set_parameters(ctrl, {});
        }

        const std::vector<double>& parameters() const
        {
            return _controller.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {
            std::cout << "update cpg" << std::endl;
            std::vector<double> angles = _controller.pos(t);
            std::cout << "angle calculated" << std::endl;
            for (size_t i = 0; i < angles.size(); i++) {
                std::cout << "boucle for" << std::endl;
                std::cout << "angle :" << angles[i] << std::endl;
                _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];
            }
            set_commands();
        }

        void set_commands()
        {
            std::cout << "set command cpg" << std::endl;
            if (_robot == nullptr)
                return;

            Eigen::VectorXd q = _robot->skeleton()->getPositions();
            Eigen::VectorXd q_err = _target_positions - q;

            double gain = 1.0 / (dart::math::constants<double>::pi() * _robot->skeleton()->getTimeStep());
            Eigen::VectorXd vel = q_err * gain;
            vel = vel.cwiseProduct(_p);

            _robot->skeleton()->setCommands(vel);
        }

    protected:
        hexapod_controller::CpgOpenLoop _controller;
        robot_t _robot;
        Eigen::VectorXd _vel_cmd;
        std::vector<int> _broken_legs;
        Eigen::VectorXd _p;
        Eigen::VectorXd _target_positions;
    };
} // namespace hexapod_dart

#endif
