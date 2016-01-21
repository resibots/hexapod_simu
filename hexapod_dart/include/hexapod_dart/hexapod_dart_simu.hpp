#ifndef HEXAPOD_DART_SIMU_HPP
#define HEXAPOD_DART_SIMU_HPP

#include <boost/parameter.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/find.hpp>

#include <dart/dart.h>
#include <dart/collision/dart/DARTCollisionDetector.h>
#include <Eigen/Core>
#include <hexapod_dart/hexapod.hpp>
#include <hexapod_dart/hexapod_control.hpp>
#include <hexapod_dart/safety_measures.hpp>
#include <hexapod_dart/descriptors.hpp>

#ifdef GRAPHIC
#include <osgDart/osgDart.h>
#endif

namespace hexapod_dart {

    BOOST_PARAMETER_TEMPLATE_KEYWORD(hexapod_control)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(safety)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(desc)

    typedef boost::parameter::parameters<boost::parameter::optional<tag::hexapod_control>,
        boost::parameter::optional<tag::safety>,
        boost::parameter::optional<tag::desc>> class_signature;

    template <typename Simu, typename robot>
    struct Refresh {
        Refresh(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_rot)
            : _simu(simu), _robot(rob), _init_pos(init_pos), _init_rot(init_rot) {}

        Simu& _simu;
        std::shared_ptr<robot> _robot;
        Eigen::Vector3d _init_pos, _init_rot;

        template <typename T>
        void operator()(T& x) const { x(_simu, _robot, _init_pos, _init_rot); }
    };

    template <class A1 = boost::parameter::void_, class A2 = boost::parameter::void_, class A3 = boost::parameter::void_>
    class HexapodDARTSimu {
    public:
        using robot_t = std::shared_ptr<Hexapod>;
        // defaults
        struct defaults {
            using hexapod_control_t = HexapodControl;
            using safety_measures_t = boost::fusion::vector<safety_measures::MaxHeight>;
            using descriptors_t = boost::fusion::vector<descriptors::DutyCycle>;
        };

        // extract the types
        using args = typename class_signature::bind<A1, A2, A3>::type;
        using hexapod_control_t = typename boost::parameter::binding<args, tag::hexapod_control, typename defaults::hexapod_control_t>::type;
        using SafetyMeasures = typename boost::parameter::binding<args, tag::safety, typename defaults::safety_measures_t>::type;
        using Descriptors = typename boost::parameter::binding<args, tag::desc, typename defaults::descriptors_t>::type;
        using safety_measures_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<SafetyMeasures>, SafetyMeasures, boost::fusion::vector<SafetyMeasures>>::type;
        using descriptors_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Descriptors>, Descriptors, boost::fusion::vector<Descriptors>>::type;

        HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) : _covered_distance(0.0),
                                                                          _energy(0.0),
                                                                          _world(std::make_shared<dart::simulation::World>()),
                                                                          _controller(ctrl, robot),
                                                                          _old_index(0),
                                                                          _break(false)
        {
            _world->getConstraintSolver()->setCollisionDetector(new dart::collision::DARTCollisionDetector());
            _robot = robot;
            // set position of hexapod
            _robot->skeleton()->setPosition(5, 0.1);
            _robot->skeleton()->setPosition(2, DART_PI);
            _add_floor();
            _world->addSkeleton(_robot->skeleton());
            _world->setTimeStep(0.015);

            std::vector<double> c_tmp(36, 0.0);
            _controller.set_parameters(c_tmp);
            _stabilize_robot(true);
            _world->setTime(0.0);
            _controller.set_parameters(ctrl);

#ifdef GRAPHIC
            _osg_world_node = new osgDart::WorldNode(_world);
            _osg_world_node->simulate(true);
            _osg_viewer.addWorldNode(_osg_world_node);
            // _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
            // full-screen
            _osg_viewer.setUpViewOnSingleScreen();
#endif
        }

        ~HexapodDARTSimu() {}

        void run(double duration = 5.0, bool continuous = false, bool chain = false)
        {
            _break = false;
            robot_t rob = this->robot();
            double old_t = _world->getTime();
            int index = _old_index;

            // TO-DO: maybe wee need better solution for this/reset them?
            static Eigen::Vector3d init_pos = rob->pos();
            static Eigen::Vector3d init_rot = rob->rot();
            static Eigen::VectorXd torques(rob->skeleton()->getNumDofs());

#ifdef GRAPHIC
            while ((_world->getTime() - old_t) < duration && !_osg_viewer.done())
#else
            while ((_world->getTime() - old_t) < duration)
#endif
            {
                _controller.update(chain ? (_world->getTime() - old_t) : _world->getTime());

                _world->step(false);

                // integrate Torque (force) over time
                Eigen::VectorXd state = rob->skeleton()->getForces().array().abs() * _world->getTimeStep();
                torques = torques + state;

                // update safety measures
                boost::fusion::for_each(_safety_measures, Refresh<HexapodDARTSimu, Hexapod>(*this, rob, init_pos, init_rot));

                if (_break) {
                    _covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }

#ifdef GRAPHIC
                auto COM = rob->skeleton()->getCOM();
                // set camera to follow hexapod
                _osg_viewer.getCameraManipulator()->setHomePosition(
                    osg::Vec3d(-0.5, 3, 1), osg::Vec3d(COM(0), COM(1), COM(2)), osg::Vec3d(0, 0, 1));
                _osg_viewer.home();
                // process next frame
                _osg_viewer.frame();
#endif

                if (index % 2 == 0) {
                    // update descriptors
                    boost::fusion::for_each(_descriptors, Refresh<HexapodDARTSimu, Hexapod>(*this, rob, init_pos, init_rot));
                }

                // roll-pitch-yaw
                auto rot_mat = dart::math::expMapRot(rob->rot() - init_rot);
                auto rpy = dart::math::matrixToEulerXYZ(rot_mat);

                _behavior_traj.push_back(rob->pos() - init_pos);
                _rotation_traj.push_back(std::round(rpy(2) * 100) / 100.0);

                ++index;
            }
            _energy += torques.sum();
            _old_index = index;

            if (!continuous) {
                if (!_stabilize_robot()) {
                    _covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }
            }

            Eigen::Vector3d stab_pos = rob->pos();
            Eigen::Vector3d stab_rot = rob->rot();

            _final_pos = stab_pos - init_pos;
            _final_rot = stab_rot - init_rot;

            _covered_distance = std::round(_final_pos(0) * 100) / 100.0;

            // roll-pitch-yaw
            _arrival_angle = std::round(dart::math::matrixToEulerXYZ(dart::math::expMapRot(_final_rot))(2) * 100) / 100.0;
        }

        robot_t robot()
        {
            return _robot;
        }

        template <typename Desc, typename T>
        void get_descriptor(T& result)
        {
            auto d = boost::fusion::find<Desc>(_descriptors);
            (*d).get(result);
        }

        double covered_distance()
        {
            return _covered_distance;
        }

        double energy()
        {
            return _energy;
        }

        double arrival_angle()
        {
            return _arrival_angle;
        }

        Eigen::Vector3d final_pos()
        {
            return _final_pos;
        }

        Eigen::Vector3d final_rot()
        {
            return _final_rot;
        }

        double step()
        {
            assert(_world != nullptr);
            return _world->getTimeStep();
        }

        void set_step(double step)
        {
            assert(_world != nullptr);
            _world->setTimeStep(step);
        }

        void stop_sim(bool disable = true)
        {
            _break = disable;
        }

        HexapodControl& controller()
        {
            return _controller;
        }

        const std::vector<Eigen::Vector3d>& pos_traj()
        {
            return _behavior_traj;
        }

        const std::vector<double>& rot_traj()
        {
            return _rotation_traj;
        }

    protected:
        bool _stabilize_robot(bool update_ctrl = false)
        {
            robot_t rob = this->robot();

            bool stabilized = false;
            int stab = 0;

            if (update_ctrl)
                _world->setTimeStep(0.001);

            for (size_t s = 0; s < 1000 && !stabilized; ++s) {
                Eigen::Vector6d prev_pose = rob->pose();

                if (update_ctrl)
                    _controller.update(_world->getTime());
                else
                    _controller.set_commands();
                _world->step();

                if ((rob->pose() - prev_pose).norm() < 1e-4)
                    stab++;
                else
                    stab = 0;
                if (stab > 30)
                    stabilized = true;
            }

            if (update_ctrl)
                _world->setTimeStep(0.015);

            return stabilized;
        }

        void _add_floor()
        {
            // We do not want 2 floors!
            if (_world->getSkeleton("floor") != nullptr)
                return;

            dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");

            // Give the floor a body
            dart::dynamics::BodyNodePtr body = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

            // Give the body a shape
            double floor_width = 10.0;
            double floor_height = 0.1;
            std::shared_ptr<dart::dynamics::BoxShape> box(
                new dart::dynamics::BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
            box->setColor(dart::Color::Gray());

            body->addVisualizationShape(box);
            body->addCollisionShape(box);

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
            body->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(floor);
        }

        std::vector<Eigen::Vector3d> _behavior_traj;
        std::vector<double> _rotation_traj;
        robot_t _robot;
        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _arrival_angle;
        double _covered_distance;
        double _energy;
        dart::simulation::WorldPtr _world;
        hexapod_control_t _controller;
        int _old_index;
        bool _break;
        safety_measures_t _safety_measures;
        descriptors_t _descriptors;
#ifdef GRAPHIC
        osg::ref_ptr<osgDart::WorldNode> _osg_world_node;
        osgDart::Viewer _osg_viewer;
#endif
    };
}

#endif
