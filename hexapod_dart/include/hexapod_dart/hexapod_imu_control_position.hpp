#ifndef HEXAPOD_DART_HEXAPOD_IMU_CONTROL_POSITION
#define HEXAPOD_DART_HEXAPOD_IMU_CONTROL_POSITION

#include <hexapod_controller/hexapod_controller_imu_position.hpp>
#include <hexapod_dart/hexapod.hpp>
#include <iostream>
namespace hexapod_dart {

    class HexapodIMUControlPos {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodIMUControlPos() {}
        HexapodIMUControlPos(const std::vector<double>& ctrl, robot_t robot)
            : _controller(ctrl, robot->broken_legs()), _robot(robot)
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

        void set_parameters(const std::vector<double>& ctrl)
        {
            _controller.set_parameters(ctrl);
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
            Eigen::VectorXd q = _robot->skeleton()->getPositions();
            //Extract yaw pitch roll of the robot thanks to COM data
            Eigen::Matrix3d rot = dart::math::expMapRot({q[0], q[1], q[2]});
            // Angle computation roll-pitch-yaw
            Eigen::Vector3d rpy = dart::math::matrixToEulerXYZ(rot);

            _controller.computeErrors(rpy(0), rpy(1));
            auto angles = _controller.pos(t);
            for (size_t i = 0; i < angles.size(); i++)
                _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];

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

            _robot->skeleton()->setCommands(vel);
        }

    protected:
        hexapod_controller::HexapodControllerImuPos _controller;
        robot_t _robot;

        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;
    };
} // namespace hexapod_dart

#endif
