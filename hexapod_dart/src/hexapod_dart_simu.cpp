#include <hexapod_dart_simu.hpp>
#include <dart/collision/dart/DARTCollisionDetector.h>

HexapodDARTSimu::HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) : _controller(ctrl, robot),
                                                                                   _covered_distance(0.0),
                                                                                   _energy(0.0),
                                                                                   _world(std::make_shared<dart::simulation::World>()),
                                                                                   _old_index(0)
{
    _world->getConstraintSolver()->setCollisionDetector(new dart::collision::DARTCollisionDetector());
    _robot = robot;
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
    _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
#endif
}

HexapodDARTSimu::~HexapodDARTSimu()
{
    // we have to clean in the good order
    _robot.reset();
    _world.reset();
}

void HexapodDARTSimu::run(double duration, bool continuous, bool chain)
{
    robot_t rob = this->robot();
    double old_t = _world->getTime();
    int index = _old_index;

    // TO-DO: maybe wee need better solution for this/reset them?
    static Eigen::Vector3d init_pos = rob->pos();
    static Eigen::Vector3d init_rot = rob->rot();

#ifdef GRAPHIC
    while ((_world->getTime() - old_t) < duration && !_osg_viewer.done())
#else
    while ((_world->getTime() - old_t) < duration)
#endif
    {
        _controller.update(chain ? (_world->getTime() - old_t) : _world->getTime());

        auto body = rob->skeleton()->getRootBodyNode();
        auto COM = rob->skeleton()->getCOM();
        // TO-DO: check also for leg collisions?
        if (body->isColliding() || std::abs(COM(2)) > 0.3) {
            _covered_distance = -10002.0;
            _arrival_angle = -10002.0;
            _direction = -10002.0;
            return;
        }

        _world->step();

#ifdef GRAPHIC
        _osg_viewer.frame();
#endif

        if (index % 2 == 0) {
            _check_duty_cycle();
        }

        Eigen::Vector3d pos = rob->pos();
        Eigen::Vector3d rot = rob->rot();

        _behavior_traj.push_back(pos);
        _rotation_traj.push_back(_min_dist_angle(std::round(rot(2) * DART_DEGREE * 100) / 100.0, std::round(init_rot(2) * DART_DEGREE * 100) / 100.0));

        ++index;
    }
    _old_index = index;

    if (!continuous) {
        if (!_stabilize_robot()) {
            _covered_distance = -10002.0;
            _arrival_angle = -10002.0;
            _direction = -10002.0;
            return;
        }
    }

    Eigen::Vector3d stab_pos = rob->pos();
    Eigen::Vector3d stab_rot = rob->rot();

    Eigen::Vector3d final_pos = stab_pos - init_pos;

    _covered_distance = std::round(final_pos(0) * 100) / 100.0;

    _arrival_angle = _min_dist_angle(std::round(stab_rot(2) * DART_DEGREE * 100) / 100.0, std::round(init_rot(2) * DART_DEGREE * 100) / 100.0);

    _direction = std::atan2(final_pos(1), final_pos(0)) * DART_DEGREE;
}

HexapodDARTSimu::robot_t HexapodDARTSimu::robot()
{
    return _robot;
}

double HexapodDARTSimu::covered_distance()
{
    return _covered_distance;
}

std::vector<double> HexapodDARTSimu::duty_cycle()
{
    std::vector<double> results;
    double sum = 0;
    for (size_t i = 0; i < _behavior_contact_0.size(); i++)
        sum += _behavior_contact_0[i];
    sum /= _behavior_contact_0.size();
    results.push_back(std::round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_1.size(); i++)
        sum += _behavior_contact_1[i];
    sum /= _behavior_contact_1.size();
    results.push_back(std::round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_2.size(); i++)
        sum += _behavior_contact_2[i];
    sum /= _behavior_contact_2.size();
    results.push_back(std::round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_3.size(); i++)
        sum += _behavior_contact_3[i];
    sum /= _behavior_contact_3.size();
    results.push_back(std::round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_4.size(); i++)
        sum += _behavior_contact_4[i];
    sum /= _behavior_contact_4.size();
    results.push_back(std::round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_5.size(); i++)
        sum += _behavior_contact_5[i];
    sum /= _behavior_contact_5.size();
    results.push_back(std::round(sum * 100) / 100.0);

    return results;
}

double HexapodDARTSimu::energy()
{
    return _energy;
}

double HexapodDARTSimu::direction()
{
    return _direction;
}

double HexapodDARTSimu::arrival_angle()
{
    return _arrival_angle;
}

Eigen::Vector3d HexapodDARTSimu::final_pos()
{
    return _final_pos;
}

double HexapodDARTSimu::step()
{
    assert(_world != nullptr);
    return _world->getTimeStep();
}

void HexapodDARTSimu::set_step(double step)
{
    assert(_world != nullptr);
    _world->setTimeStep(step);
}

HexapodControl& HexapodDARTSimu::controller()
{
    return _controller;
}

const std::vector<Eigen::Vector3d>& HexapodDARTSimu::pos_traj()
{
    return _behavior_traj;
}

const std::vector<double>& HexapodDARTSimu::rot_traj()
{
    return _rotation_traj;
}

const std::vector<double>& HexapodDARTSimu::contact(int i)
{
    switch (i) {
    case 0:
        return _behavior_contact_0;
        break;
    case 1:
        return _behavior_contact_1;
        break;
    case 2:
        return _behavior_contact_2;
        break;
    case 3:
        return _behavior_contact_3;
        break;
    case 4:
        return _behavior_contact_4;
        break;
    case 5:
        return _behavior_contact_5;
        break;
    }
    assert(false);
    return _behavior_contact_0;
}

bool HexapodDARTSimu::_stabilize_robot(bool update_ctrl)
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

void HexapodDARTSimu::_add_floor()
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

double HexapodDARTSimu::_min_dist_angle(double a1, double a2)
{
    // compute minimum signed distance between 2 angles
    double res = a1 - a2;
    // res = (res + 180) % 360 - 180;
    // mod = (a, n) -> a - floor(a/n) * n
    res = res + 180.0;
    res = res - std::floor(res / 360.0) * 360.0;
    res = res - 180.0;

    // we want [-180,180] range
    while (res < -180)
        res += 360;
    while (res > 180)
        res -= 360;
    return res;
}

void HexapodDARTSimu::_check_duty_cycle()
{
    auto rob = this->robot();

    for (unsigned i = 0; i < 6; ++i) {
        std::string leg_name = "leg_" + std::to_string(i) + "_3";
        dart::dynamics::BodyNodePtr tmp;
        for (int j = 0; j < rob->skeleton()->getNumBodyNodes(); j++) {
            auto bd = rob->skeleton()->getBodyNode(j);
            if (leg_name == bd->getName())
                tmp = bd;
        }
        switch (i) {
        case 0:
            if (rob->is_broken(i)) {
                _behavior_contact_0.push_back(0);
            }
            else {
                _behavior_contact_0.push_back(tmp->isColliding());
            }
            break;
        case 1:
            if (rob->is_broken(i)) {
                _behavior_contact_1.push_back(0);
            }
            else {
                _behavior_contact_1.push_back(tmp->isColliding());
            }
            break;
        case 2:
            if (rob->is_broken(i)) {
                _behavior_contact_2.push_back(0);
            }
            else {
                _behavior_contact_2.push_back(tmp->isColliding());
            }
            break;
        case 3:
            if (rob->is_broken(i)) {
                _behavior_contact_3.push_back(0);
            }
            else {
                _behavior_contact_3.push_back(tmp->isColliding());
            }
            break;
        case 4:
            if (rob->is_broken(i)) {
                _behavior_contact_4.push_back(0);
            }
            else {
                _behavior_contact_4.push_back(tmp->isColliding());
            }
            break;
        case 5:
            if (rob->is_broken(i)) {
                _behavior_contact_5.push_back(0);
            }
            else {
                _behavior_contact_5.push_back(tmp->isColliding());
            }
            break;
        }
    }
}
