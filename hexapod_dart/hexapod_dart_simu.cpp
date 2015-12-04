#include <hexapod_dart_simu.hpp>

HexapodDARTSimu::HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) : _controller(ctrl, robot->broken_legs()),
                                                                                   _covered_distance(0.0),
                                                                                   _energy(0.0),
                                                                                   _world(std::make_shared<dart::simulation::World>()),
                                                                                   _old_t(0.0),
                                                                                   _old_index(0)
{
    // TO-DO Initialization of world/robot
    _robot = robot;
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
    double t = _old_t;
    std::vector<double> angles;
    int index = _old_index;
    while ((t - _old_t) < duration) {
        angles = _controller.pos(chain ? (t - _old_t) : t);
        // TO-DO: Add actual controller
        // rob->move_joints(angles);

        // TO-DO: check if robot base collides with ground

        if (index % 2 == 0)
            // TO-DO: get contacts for feet

            // TO-DO: get COM position and push it in _behavior_traj
            // TO-DO: get COM rotation and push it in _rotation_traj

            t += _step;
        _world->step();

        ++index;
    }
    _old_t = t;
    _old_index = index;

    // TO-DO: compute covered_distance, arrival_angle and stabilize robot
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
    return _step;
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
