#ifndef HEXAPOD_DART_HEXAPOD_CPG_CONTROL
#define HEXAPOD_DART_HEXAPOD_CPG_CONTROL

#include "cpg.hpp"
#include <hexapod_dart/hexapod.hpp>
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

            //Create the joint_position vector for the cpg object
            std::vector<float> joint_position;
            for (size_t i = 0; i < 18; i++) {
                joint_position.push_back(q(i + 6)); //Don't take the 6 first data wich is for the COM. Take only joint angles
            }

            /*compute CPG cmd, without leveling the body*/
            _cpg.computeCPGcmd();

            /*modify cpg cmd to take into account the roll and pitch, close the loop*/
            std::vector<float> cmd = _cpg.computeErrors(rpy(0), rpy(1), joint_position);

            for (size_t i = 0; i < 6 + 18; i++) {
                _vel_cmd(i) = 0; //Don't move the COM ?
            }

            int index = 0;
            bool is_broken = false;
            for (size_t i = 0; i < 6; i++) {
                // is_broken = false;
                // for (int j : _broken_legs) {
                // if (i == j) {
                // is_broken = true;
                // }
                // }
                // if (is_broken == false) {
                _vel_cmd(i + 6) = cmd[index]; //First joint of the leg i (xcommand)
                index++;
                _vel_cmd(i + 12) = cmd[index]; //Second joint of the leg i (ycommand)
                index++;
                _vel_cmd(i + 18) = cmd[index]; //Third joint of the leg i (ycommand)
                index++;
                // }
            }

            set_commands();
        }

        void set_commands()
        {
            if (_robot == nullptr)
                return;

            _robot->skeleton()->setCommands(_vel_cmd);
        }

    protected:
        cpg::CPG _cpg;
        robot_t _robot;
        Eigen::VectorXd _vel_cmd;
        std::vector<int> _broken_legs;
    };
} // namespace hexapod_dart

#endif
