#ifndef HEXAPOD_DART_HEXAPOD_IMU_CONTROL
#define HEXAPOD_DART_HEXAPOD_IMU_CONTROL

#include <hexapod_controller/hexapod_controller_imu.hpp>
#include <hexapod_dart/hexapod.hpp>
namespace hexapod_dart {

    class HexapodIMUControl {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodIMUControl()
        {
            _t_prev_ = 0;
        }
        HexapodIMUControl(const std::vector<double>& ctrl, robot_t robot)
            : _controller(ctrl, robot->broken_legs()), _robot(robot)
        {
            _t_prev_ = 0;
            //_robot = robot;
            _broken_legs = robot->broken_legs();

            _target_positions = _robot->skeleton()->getPositions();

            size_t dof = _robot->skeleton()->getNumDofs();

            _p = Eigen::VectorXd::Zero(dof);
            _vel_cmd = Eigen::VectorXd::Zero(dof);

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

            bool is_broken = false; // We suppose at first that no leg is missing
            int i_broken = 0;

            std::vector<double> angles = _controller.pos(t);
            for (size_t i = 0; i < angles.size(); i++)
                _target_positions(i + 6) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];

            //Create the joint_position vector for the cpg object, it needs the position of every leg
            std::vector<float> joint_position;
            joint_position.resize(18, 0.0); //It will be zero if the leg is broken
            for (size_t i = 0; i < 6; i++) {
                is_broken = false;
                //Check if the i th  leg is broker
                for (size_t j : _broken_legs) {
                    if (i == j) {
                        is_broken = true;
                    }
                }
                if (is_broken == false) {
                    //if the leg is not broken fill in the joint_position in the right orger
                    //The +6 offset in q is because we don't take the 6 first data wich is for the COM. Take only joint angles
                    joint_position[i] = q(3 * i_broken + 6); //joint_position[i] is the first servo of leg i (shoulder joint)
                    joint_position[i + 6] = q(3 * i_broken + 7); //joint_position[i+6] is the second servo of leg i
                    joint_position[i + 12] = q(3 * i_broken + 8); //joint_position[i+12] is the third servo of leg i
                    i_broken++;
                }
            }
            double loop_rate = t - _t_prev_;
            _t_prev_ = t;
            std::vector<float> cmd = _controller.computeErrors(rpy(0), rpy(1), q, _target_positions, loop_rate);

            for (size_t i = 0; i < 6 + 3 * (6 - _broken_legs.size()); i++) {
                _vel_cmd(i) = 0; //Don't move the COM ?
            }

            int index = 0;
            i_broken = 0;
            for (size_t i = 0; i < 6; i++) {
                is_broken = false;
                for (size_t j : _broken_legs) {
                    if (i == j) {
                        is_broken = true;
                        //index += 3;
                        //i_broken++;
                    }
                }
                if (is_broken == false) {
                    _vel_cmd(3 * i_broken + 6) = cmd[index]; //First joint of the leg i (xcommand)
                    index++;
                    _vel_cmd(3 * i_broken + 7) = cmd[index]; //Second joint of the leg i (ycommand)
                    index++;
                    _vel_cmd(3 * i_broken + 8) = cmd[index]; //Third joint of the leg i (ycommand)
                    index++;
                    i_broken++;
                }
            }

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
            _vel_cmd = _vel_cmd.cwiseProduct(_p);
            // for (size_t i = 0; i < 6 + 3 * (6 - _broken_legs.size()); i++) {
            //     _vel_cmd(i) = 0; //Don't move the COM ?
            // }
            _robot->skeleton()->setCommands(_vel_cmd);
        }

    protected:
        hexapod_controller::HexapodControllerImu _controller;
        robot_t _robot;
        Eigen::VectorXd _vel_cmd;
        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;
        double _t_prev_;
        std::vector<int> _broken_legs;
    };
} // namespace hexapod_dart

#endif
