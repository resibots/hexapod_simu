#ifndef HEXAPOD_DART_SIMU_HPP
#define HEXAPOD_DART_SIMU_HPP

#include <dart/dart.h>
#include <Eigen/Core>
#include <hexapod_controller_simple.hpp>

class Simu {
public:
    static constexpr double step = 0.015;
    typedef std::vector<double> ctrl_t;

    Simu(const ctrl_t& ctrl, const std::vector<int>& broken_legs);

    ~Simu();

    void run(double duration = 5.0, bool continuous = false, bool chain = false);

    void next_step();

    double covered_distance();

    std::vector<double> get_duty_cycle();

    double energy();
    double direction();
    double arrival_angle();
    Eigen::Vector3d final_pos();

    HexapodControllerSimple& controller() { return _controller; }

    const std::vector<Eigen::Vector3d>& get_traj();
    const std::vector<double>& get_rot_traj();

    const std::vector<double>& get_contact(int i);

protected:
    bool _stabilize_robot();

    void _make_robot_init();

    std::vector<Eigen::Vector3d> _behavior_traj;
    std::vector<double> _rotation_traj;
    std::vector<double> _behavior_contact_0;
    std::vector<double> _behavior_contact_1;
    std::vector<double> _behavior_contact_2;
    std::vector<double> _behavior_contact_3;
    std::vector<double> _behavior_contact_4;
    std::vector<double> _behavior_contact_5;
    HexapodControllerSimple _controller;
    Eigen::Vector3d _final_pos;
    double _direction;
    double _arrival_angle;
    double _covered_distance;
    double _energy;
    dart::simulation::WorldPtr _world;
    bool _transf;
    double _old_t;
    int _old_index;
    bool _init;
    const double _angle;
};

#endif