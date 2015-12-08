#ifndef HEXAPOD_ROBDYN_SIMU_HPP
#define HEXAPOD_ROBDYN_SIMU_HPP

#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>

#ifdef GRAPHIC
#include <renderer/osg_visitor.hh>
#endif

#include <hexapod.hpp>
#include <hexapod_controller_simple.hpp>
#define MAX_ANG_VEL 11.9380521
#define DYN2RAD 0.00511326929
#define SAMPLING_FREQUENCY 20

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/filesystem.hpp>
#include <boost/config.hpp>


class HexapodRobdynSimu {
public:
    BOOST_STATIC_CONSTEXPR double step = 0.015;
    typedef boost::shared_ptr<robot::Hexapod> robot_t;
    typedef std::vector<double> ctrl_t;

    HexapodRobdynSimu(const ctrl_t& ctrl, const robot_t& robot, bool transf = false, double angle = 0);

    ~HexapodRobdynSimu();

    void run(double duration = 5.0, bool continuous = false, bool chain = false);

    void next_step();

    robot_t robot();

    double covered_distance();

    std::vector<double> get_duty_cycle();

    double energy();
    double direction();
    double arrival_angle();
    Eigen::Vector3d final_pos();

    HexapodControllerSimple& controller() { return _controller; }

    void write_contact(std::string const name);

    void write_traj(std::string const name);

    const std::vector<Eigen::Vector3d>& get_traj();
    const std::vector<double>& get_rot_traj();

    const std::vector<double>& get_contact(int i);

protected:
    bool _stabilize_robot();

    void _make_robot_init();
    boost::filesystem::path _create_database_folder();
    boost::filesystem::path _create_exp_folder();

    template <typename Data_t>
    void _write_data(Data_t data, std::string name)
    {
        //boost::filesystem::path expDir = _create_exp_folder();

        std::ofstream ofs((_writing_path.string() + std::string("/") + name).c_str());
        // save data to archive
        {
            boost::archive::text_oarchive oa(ofs);
            // write class instance to archive
            oa << data;
            // archive and stream closed when destructors are called
        }
    }

    template <typename R>
    Eigen::VectorXd _get_state(const R& rob)
    {
        Eigen::VectorXd act_state = Eigen::VectorXd::Zero(rob->servos().size() + rob->motors().size());
        size_t k = 0;
        for (size_t i = 0; i < rob->servos().size(); ++i, ++k)
            act_state[k] = rob->servos()[i]->get_angle(2);
        for (size_t i = 0; i < rob->motors().size(); ++i, ++k)
            act_state[k] = rob->motors()[i]->get_pos();

        return act_state;
    }

    std::vector<Eigen::Vector3d> _behavior_traj;
    std::vector<double> _rotation_traj;
    std::vector<double> _behavior_contact_0;
    std::vector<double> _behavior_contact_1;
    std::vector<double> _behavior_contact_2;
    std::vector<double> _behavior_contact_3;
    std::vector<double> _behavior_contact_4;
    std::vector<double> _behavior_contact_5;
    HexapodControllerSimple _controller;
    robot_t _robot;
    Eigen::Vector3d _final_pos;
    double _direction;
    double _arrival_angle;
    double _covered_distance;
    double _energy;
    boost::shared_ptr<ode::Environment_hexa> _env;
    bool _transf;
    double _old_t;
    int _old_index;
    bool _init;
    const double _angle;

#ifdef GRAPHIC
    renderer::OsgVisitor _visitor;
#endif
    boost::filesystem::path _writing_path;
};

#endif