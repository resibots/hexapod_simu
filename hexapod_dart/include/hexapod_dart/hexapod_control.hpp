#ifndef HEXAPOD_DART_HEXAPOD_CONTROL
#define HEXAPOD_DART_HEXAPOD_CONTROL

#include <hexapod_dart/hexapod.hpp>
#include <hexapod_controller/hexapod_controller_simple.hpp>

namespace hexapod_dart {

    class HexapodControl {
    public:
        using robot_t = std::shared_ptr<Hexapod>;

        HexapodControl() {}
        HexapodControl(const std::vector<double>& ctrl, robot_t robot);

        void set_parameters(const std::vector<double>& ctrl);
        std::vector<double> parameters();

        robot_t robot();

        void update(double t);

        void set_commands();

    protected:
        hexapod_controller::HexapodControllerSimple _controller;
        robot_t _robot;

        Eigen::VectorXd _target_positions;
        Eigen::VectorXd _p;
    };
}

#endif
