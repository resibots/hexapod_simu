#ifndef HEXAPOD_DART_HEXAPOD_CONTROL_CPG
#define HEXAPOD_DART_HEXAPOD_CONTROL_CPG

#include <hexapod_controller/cpg_open_loop.hpp>
#include <hexapod_dart/hexapod.hpp>
#include <iostream>
namespace hexapod_dart {

    class HexapodControlCPG {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodControlCPG() { _controller.computeTrajectory(30); }
        HexapodControlCPG(const std::vector<double>& ctrl, robot_t robot)
        {
            _controller.computeTrajectory(30);
            _broken_legs = robot->broken_legs();
            _target_positions = _robot->skeleton()->getPositions();
            _robot = robot;
            _controller.set_parameters(ctrl, _broken_legs);
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

        void set_parameters(const std::vector<double>& ctrl)
        {
            _controller.set_parameters(ctrl, _broken_legs);
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
            std::cout << "IN UPDATE" << std::endl;

            auto angles = _controller.pos(t);
            std::cout << "pos compute" << std::endl;
            for (size_t i = 0; i < angles.size(); i++)
                std::cout << "retarget" << std::endl;
            _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];
            std::cout << "retarget done" << std::endl;
            set_commands();
        }

        void set_commands()
        {
            if (_robot == nullptr)
                return;

            Eigen::VectorXd q = _robot->skeleton()->getPositions();
            Eigen::VectorXd q_err = _target_positions - q;

            double gain = 1.0 / (dart::math::constants<double>::pi() * _robot->skeleton()->getTimeStep());
            Eigen::VectorXd vel = q_err * gain;
            vel = vel.cwiseProduct(_p);
            std::cout << "set command" << std::endl;
            _robot->skeleton()->setCommands(vel);
        }

    protected:
        hexapod_controller::CpgOpenLoop _controller;
        robot_t _robot;

        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;
        std::vector<int> _broken_legs;
    };
} // namespace hexapod_dart

#endif
