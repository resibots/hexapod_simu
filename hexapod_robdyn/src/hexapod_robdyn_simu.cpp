#ifdef GRAPHIC
#include <renderer/osg_visitor.hh>
#endif

#include <numeric>
#include <hexapod_robdyn/hexapod_robdyn_simu.hpp>

using namespace hexapod_robdyn;

HexapodRobdynSimu::HexapodRobdynSimu(const ctrl_t& ctrl, const robot_t& robot, bool transf, double angle) : _controller(ctrl, robot->broken_legs()),
                                                                                                            _covered_distance(0.0),
                                                                                                            _energy(0.0),
                                                                                                            _env(new ode::Environment_hexa(angle)),
                                                                                                            _transf(transf),
                                                                                                            _old_t(0.0),
                                                                                                            _old_index(0),
                                                                                                            _init(false),
                                                                                                            _angle(angle)
{
    assert(ctrl.size() == 36);

    _robot = robot->clone(*_env);

    if (_transf) {
        _writing_path = _create_exp_folder();
        _write_data(ctrl, "ctrl");
    }
}

HexapodRobdynSimu::~HexapodRobdynSimu()
{
    // we have to clean in the good order
    _robot.reset();
    _env.reset();
}

void HexapodRobdynSimu::run(double duration, bool continuous, bool chain)
{
    try {
        _env->set_gravity(0, 0, -9.81);

        if (!_init)
            _make_robot_init();
#ifdef GRAPHIC
        _robot->accept(_visitor);
#endif

        robot_t rob = this->robot();
        Eigen::Vector3d rot = rob->rot();
        Eigen::VectorXd act_state = _get_state(rob);

        // Eigen::Vector3d prev_pos = rob->pos();

        double t = _old_t;
        std::vector<double> angles;
        int index = _old_index;
#ifdef GRAPHIC
        while ((t - _old_t) < duration && !_visitor.done())
#else
        while ((t - _old_t) < duration)
#endif
        {
            angles = _controller.pos(chain ? (t - _old_t) : t);
            rob->move_joints(angles);

            Eigen::VectorXd new_state = _get_state(rob);
            _energy += (act_state - new_state).array().abs().sum();
            act_state = new_state;
            if (rob->bodies()[0]->get_in_contact() || _env->get_colision_between_legs()) {
                _covered_distance = -10002;
                return;
            }
            int nbCassee = 0;
            if (index % 2 == 0)
                for (unsigned i = 0; i < 6; ++i) {
                    switch (i) {
                    case 0:
                        if (rob->is_broken(i)) {
                            _behavior_contact_0.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_0.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    case 1:
                        if (rob->is_broken(i)) {
                            _behavior_contact_1.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_1.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    case 2:
                        if (rob->is_broken(i)) {
                            _behavior_contact_2.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_2.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    case 3:
                        if (rob->is_broken(i)) {
                            _behavior_contact_3.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_3.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    case 4:
                        if (rob->is_broken(i)) {
                            _behavior_contact_4.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_4.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    case 5:
                        if (rob->is_broken(i)) {
                            _behavior_contact_5.push_back(0);
                            nbCassee++;
                        }
                        else {
                            _behavior_contact_5.push_back(rob->bodies()[(i - nbCassee) * 3 + 3]->get_in_contact());
                        }
                        break;
                    }
                }
            _behavior_traj.push_back(rob->pos());
            rot = rob->rot();
            _rotation_traj.push_back(atan2(cos(rot[2]) * sin(rot[1]) * sin(rot[0]) + sin(rot[2]) * cos(rot[0]), cos(rot[2]) * cos(rot[1])) * 180 / M_PI);

            t += step;
            next_step();

            ++index;
        }
        _old_t = t;
        _old_index = index;

        if (fabs(_angle) < 0.01 && !continuous) {
            _stabilize_robot();
        }
        Eigen::Vector3d next_pos = rob->pos();

        _final_pos = next_pos;

        _covered_distance = round(next_pos[1] * 100) / 100.0f;

        if (fabs(_covered_distance) > 10) {
            _covered_distance = -10002;
        }

        _direction = atan2(-next_pos[0], next_pos[1]) * 180 / M_PI;
        rot = rob->rot();
        _arrival_angle = atan2(cos(rot[2]) * sin(rot[1]) * sin(rot[0]) + sin(rot[2]) * cos(rot[0]), cos(rot[2]) * cos(rot[1])) * 180 / M_PI;
        while (_arrival_angle < -180)
            _arrival_angle += 360;
        while (_arrival_angle > 180)
            _arrival_angle -= 360;

        if (_transf) {
            std::vector<std::vector<double>> contacts;
            contacts.push_back(_behavior_contact_0);
            contacts.push_back(_behavior_contact_1);
            contacts.push_back(_behavior_contact_2);
            contacts.push_back(_behavior_contact_3);
            contacts.push_back(_behavior_contact_4);
            contacts.push_back(_behavior_contact_5);

            _write_data(contacts, "contacts");
            std::vector<double> angles;
            angles.push_back(rob->rot()[0]);
            angles.push_back(rob->rot()[1]);
            angles.push_back(rob->rot()[2]);

            _write_data(angles, "angles");
            _write_data(_covered_distance, "fit");
        }
    }
    catch (int e) {
        std::cout << "An exception occurred. Exception Nr. " << e << std::endl;
        _covered_distance = -10002;
    }

    // #ifdef GRAPHIC
    //         write_contact("contact_simu.txt");
    //         write_traj("traj_simu.txt");
    // #endif
}

void HexapodRobdynSimu::next_step()
{
    _robot->next_step(step);
    _env->next_step(step);
#ifdef GRAPHIC
    _visitor.update();
    usleep(1e4);
#endif
}

HexapodRobdynSimu::robot_t HexapodRobdynSimu::robot()
{
    return _robot;
}

double HexapodRobdynSimu::covered_distance()
{
    return _covered_distance;
}

std::vector<double> HexapodRobdynSimu::get_duty_cycle()
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

double HexapodRobdynSimu::energy()
{
    return _energy;
}

double HexapodRobdynSimu::direction()
{
    return _direction;
}

double HexapodRobdynSimu::arrival_angle()
{
    return _arrival_angle;
}

Eigen::Vector3d HexapodRobdynSimu::final_pos()
{
    return _final_pos;
}

void HexapodRobdynSimu::write_contact(std::string const name)
{

    std::ofstream workingFile(name.c_str());

    if (workingFile) {
        for (size_t i = 0; i < _behavior_contact_0.size(); i++) {
            workingFile << _behavior_contact_0[i] << " " << _behavior_contact_1[i] << " " << _behavior_contact_2[i] << " " << _behavior_contact_3[i] << " " << _behavior_contact_4[i] << " " << _behavior_contact_5[i] << std::endl;
        }
    }
    else {
        std::cout << "ERROR: Impossible to open the file." << std::endl;
    }
}

void HexapodRobdynSimu::write_traj(std::string const name)
{

    std::ofstream workingFile(name.c_str());

    if (workingFile) {
        for (size_t i = 0; i < _behavior_traj.size(); i++) {
            workingFile << _behavior_traj[i][0] << " " << _behavior_traj[i][1] << " " << _behavior_traj[i][2] << std::endl;
        }
    }
    else {
        std::cout << "ERROR: Impossible to open the file." << std::endl;
    }
}

const std::vector<Eigen::Vector3d>& HexapodRobdynSimu::get_traj()
{
    return _behavior_traj;
}

const std::vector<double>& HexapodRobdynSimu::get_rot_traj()
{
    return _rotation_traj;
}

const std::vector<double>& HexapodRobdynSimu::get_contact(int i)
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

bool HexapodRobdynSimu::_stabilize_robot()
{
    robot_t rob = this->robot();

    //_controller.moveRobot(rob,0);
    // low gravity to slow things down (eq. smaller timestep?)
    _env->set_gravity(0, 0, -9.81);
    bool stabilized = false;
    int stab = 0;

    for (size_t s = 0; s < 1000 && !stabilized; ++s) {
        Eigen::Vector3d prev_pos = rob->pos();

        next_step();
        if ((rob->pos() - prev_pos).norm() < 1e-4)
            stab++;
        else
            stab = 0;
        if (stab > 30)
            stabilized = true;
    }
    _env->set_gravity(0, 0, -9.81);
    return (stabilized);
}

void HexapodRobdynSimu::_make_robot_init()
{
    robot_t rob = this->robot();
    Eigen::Vector3d rot = rob->rot();
    _arrival_angle = atan2(cos(rot[2]) * sin(rot[1]) * sin(rot[0]) + sin(rot[2]) * cos(rot[0]), cos(rot[2]) * cos(rot[1])) * 180 / M_PI;

    Eigen::Vector3d target_pos(0, 2, 0.2);

    std::vector<double> angles = _controller.pos(0);
    rob->move_joints(angles);

    _old_t = 0.0;
    _energy = 0.0;
    _covered_distance = 0.0;
    _init = true;
    _old_index = 0;
}

boost::filesystem::path HexapodRobdynSimu::_create_database_folder()
{
    boost::filesystem::path thePath = boost::filesystem::current_path();
    boost::filesystem::path newDir = thePath / "database";

    if (!boost::filesystem::exists(newDir) || !boost::filesystem::is_directory(newDir)) // does p actually exist?
    {

        bool bDidCreate = boost::filesystem::create_directory(newDir);

        if (!bDidCreate)
            std::cout << "Databse's directory creation failed!" << std::endl;
    }

    return newDir;
}

boost::filesystem::path HexapodRobdynSimu::_create_exp_folder()
{
    struct tm today;
    time_t maintenant;

    time(&maintenant);
    today = *localtime(&maintenant);
    std::ostringstream oss;
    oss << today.tm_year + 1900 << "-" << today.tm_mon + 1 << "-" << today.tm_mday << "_" << today.tm_hour << "_" << today.tm_min << "_" << today.tm_sec;

    // create and open a character archive for output

    boost::filesystem::path newDir = _create_database_folder();

    newDir = newDir / oss.str().c_str();

    if (!boost::filesystem::exists(newDir) || !boost::filesystem::is_directory(newDir)) // does p actually exist?
    {

        bool bDidCreate = boost::filesystem::create_directory(newDir);

        if (!bDidCreate)
            std::cout << "Exp's directory creation failed!" << std::endl;
    }
    return newDir;
}
