#include <hexapod_dart_simu.hpp>

HexapodDARTSimu::HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) : _controller(ctrl, robot),
                                                                                   _covered_distance(0.0),
                                                                                   _energy(0.0),
                                                                                   _world(std::make_shared<dart::simulation::World>()),
                                                                                   _old_index(0)
{
    _robot = robot;
    _add_floor();
    _world->addSkeleton(_robot->skeleton());
    _world->setTimeStep(0.015);
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
    while ((_world->getTime() - old_t) < duration) {
        // check if world's time can be used
        _controller.update(chain ? (_world->getTime() - old_t) : _world->getTime());
        // TO-DO: check if robot base collides with ground
        _world->step();

        if (index % 2 == 0) {
            // TO-DO: get contacts for feet
        }

        auto pos_and_rot = rob->skeleton()->getPositions();

        Eigen::Vector3d pos = {pos_and_rot(3), pos_and_rot(4), pos_and_rot(5)};
        Eigen::Vector3d rot = {pos_and_rot(0), pos_and_rot(1), pos_and_rot(2)};

        _behavior_traj.push_back(pos);
        _rotation_traj.push_back(atan2(cos(rot[2]) * sin(rot[1]) * sin(rot[0]) + sin(rot[2]) * cos(rot[0]), cos(rot[2]) * cos(rot[1])) * 180 / M_PI);

        ++index;
    }
    _old_index = index;

    if (!continuous)
        _stabilize_robot();

    auto pos_and_rot = rob->skeleton()->getPositions();

    Eigen::Vector3d pos = {pos_and_rot(3), pos_and_rot(4), pos_and_rot(5)};
    Eigen::Vector3d rot = {pos_and_rot(0), pos_and_rot(1), pos_and_rot(2)};

    // TO-DO: compute covered_distance, arrival_angle
    _covered_distance = pos(0);
    _arrival_angle = atan2(cos(rot[2]) * sin(rot[1]) * sin(rot[0]) + sin(rot[2]) * cos(rot[0]), cos(rot[2]) * cos(rot[1])) * 180 / M_PI;
}

HexapodDARTSimu::robot_t HexapodDARTSimu::robot()
{
    return _robot;
}

double HexapodDARTSimu::covered_distance()
{
    return _covered_distance;
}

std::vector<double> HexapodDARTSimu::get_duty_cycle()
{
    std::vector<double> results;
    double sum = 0;
    for (size_t i = 0; i < _behavior_contact_0.size(); i++)
        sum += _behavior_contact_0[i];
    sum /= _behavior_contact_0.size();
    results.push_back(round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_1.size(); i++)
        sum += _behavior_contact_1[i];
    sum /= _behavior_contact_1.size();
    results.push_back(round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_2.size(); i++)
        sum += _behavior_contact_2[i];
    sum /= _behavior_contact_2.size();
    results.push_back(round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_3.size(); i++)
        sum += _behavior_contact_3[i];
    sum /= _behavior_contact_3.size();
    results.push_back(round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_4.size(); i++)
        sum += _behavior_contact_4[i];
    sum /= _behavior_contact_4.size();
    results.push_back(round(sum * 100) / 100.0);

    sum = 0;
    for (size_t i = 0; i < _behavior_contact_5.size(); i++)
        sum += _behavior_contact_5[i];
    sum /= _behavior_contact_5.size();
    results.push_back(round(sum * 100) / 100.0);

    // TODOOOO
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
    assert(_world!=nullptr);
    return _world->getTimeStep();
}

void HexapodDARTSimu::set_step(double step)
{
    assert(_world!=nullptr);
    _world->setTimeStep(step);
}

HexapodControl& HexapodDARTSimu::controller()
{
    return _controller;
}

const std::vector<Eigen::Vector3d>& HexapodDARTSimu::get_traj()
{
    return _behavior_traj;
}

const std::vector<double>& HexapodDARTSimu::get_rot_traj()
{
    return _rotation_traj;
}

const std::vector<double>& HexapodDARTSimu::get_contact(int i)
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

bool HexapodDARTSimu::_stabilize_robot()
{
    // TO-DO: stabilize robot
    return true;
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
    box->setColor(dart::Color::Black());

    body->addVisualizationShape(box);
    body->addCollisionShape(box);

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    _world->addSkeleton(floor);
}
