#ifndef HEXAPOD_ROBDYN_HEXAPOD_HPP
#define HEXAPOD_ROBDYN_HEXAPOD_HPP

// For M_PI constant
#define _USE_MATH_DEFINES

#include <robot/robot.hh>
#include <ode/environment_hexa.hh>

namespace robot {
    class Hexapod : public Robot {
    public:
        Hexapod(ode::Environment_hexa& env, const Eigen::Vector3d& pos, std::vector<int> broken_legs);

        Hexapod(const Hexapod& o, ode::Environment_hexa& env);

        boost::shared_ptr<Hexapod> clone(ode::Environment_hexa& env) const;

        void move_joints(const std::vector<double>& angles);

        bool is_broken(int leg);
        void set_broken(const std::vector<int>& broken_legs);
        std::vector<int> broken_legs();

    protected:
        void _build(ode::Environment_hexa& env, const Eigen::Vector3d& pos);

        std::vector<int> _broken_legs;
    };
}

#endif