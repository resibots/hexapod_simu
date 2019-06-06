#ifndef HEXAPOD_DART_HEXAPOD_CPG_CONTROL
#define HEXAPOD_DART_HEXAPOD_CPG_CONTROL

#include <hexapod_controller/cpg.hpp>
#include <hexapod_dart/hexapod.hpp>
#include <iostream>
namespace hexapod_dart {

    class HexapodCPGControl {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodCPGControl() {}
        HexapodCPGControl(const std::vector<double>& ctrl, robot_t robot)
        {

            _broken_legs = robot->broken_legs();

            _cpg.set_parameters(ctrl);

            _robot = robot;

            size_t dof = _robot->skeleton()->getNumDofs();

            _vel_cmd = Eigen::VectorXd::Zero(dof);
        }

        void set_parameters(const std::vector<double>& ctrl)
        {
            _cpg.set_parameters(ctrl);
        }

        const std::vector<double>& parameters() const
        {
            return _cpg.parameters();
        }

        robot_t robot()
        {
            return _robot;
        }

        void update(double t)
        {

            //Extract joint positions from the robot skeletton
            //The 6 first data are the COM rotation and position
            Eigen::VectorXd q = _robot->skeleton()->getPositions();

            //Extract yaw pitch roll of the robot thanks to COM data
            Eigen::Matrix3d rot = dart::math::expMapRot({q[0], q[1], q[2]});
            // Angle computation roll-pitch-yaw
            Eigen::Vector3d rpy = dart::math::matrixToEulerXYZ(rot);

            bool is_broken = false; // We suppose at first that no leg is missing
            int i_broken = 0;

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

            /*compute CPG cmd, without leveling the body*/
            _cpg.computeCPGcmd();

            /*modify cpg cmd to take into account the roll and pitch, close the loop*/
            std::vector<float> cmd = _cpg.computeErrors(0, 0, joint_position); //rpy(0) rpy(1)
            // if (cmd.size() == 19) {
            //     std::cout << "INTEGRATION HAS DIVERGED : sending 0 commands" << std::endl;
            // }
            // for (auto& c : cmd) {
            //     std::cout << " " << c << " ";
            // }
            // std::cout << "\n";

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
                        index += 3;
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

            // std::cout << "vel cmd \n";
            // for (int i = 0; i < _robot->skeleton()->getNumDofs(); i++) {
            //     std::cout << i << " " << _vel_cmd(i) << "\n";
            // }
            // std::cout << "\n";
            // std::cout << "vel : " << std::endl;
            // for (size_t i = 0; i < 18; i++)
            //     std::cout << _vel_cmd(6 + i) << std::endl;

            double gain = 1.0 / (dart::math::constants<double>::pi() * _robot->skeleton()->getTimeStep());
            Eigen::VectorXd vel = _vel_cmd * 1000;
            gain;
            _robot->skeleton()->setCommands(vel);
        }

    protected:
        cpg::CPG _cpg;
        robot_t _robot;
        Eigen::VectorXd _vel_cmd;
        std::vector<int> _broken_legs;
    };
} // namespace hexapod_dart

#endif
