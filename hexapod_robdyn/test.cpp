#include <iostream>
#include <simu.hpp>

int main()
{
	dInitODE();
	auto global_env = boost::shared_ptr<ode::Environment_hexa>(new ode::Environment_hexa(0));
	std::vector<int> brk = {};

    auto global_robot = boost::shared_ptr<robot::Hexapod>(new robot::Hexapod(*global_env, Eigen::Vector3d(0, 0, 0.5), brk));

    double step = 0.001;

    // low gravity to slow things down (eq. smaller timestep?)
    global_env->set_gravity(0, 0, -9.81);
    bool stabilized = false;
    int stab = 0;
    for (size_t s = 0; s < 1000 && !stabilized; ++s) {

        Eigen::Vector3d prev_pos = global_robot->pos();
        global_robot->next_step(step);
        global_env->next_step(step);

        if ((global_robot->pos() - prev_pos).norm() < 1e-4)
            stab++;
        else
            stab = 0;
        if (stab > 100)
            stabilized = true;
    }

    std::vector<double> ctrl;
    ctrl = {1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5};

    Simu simu(ctrl, global_robot);
    for(int i=0;i<167;i++)
        simu.run(0.015, true);

    ctrl = {1.0, 0.55, 0.45, 0.9, 0.6, 0.15, 0.95, 0.65, 0.15, 0.25, 0.85, 0.35, 1.0, 0.2, 0.6, 0.5, 0.05, 0.6, 1.0, 1.0, 0.4, 0.9, 0.75, 0.3, 1.0, 0.7, 0.55, 0.8, 0.25, 0.3, 1.0, 0.15, 0.3, 0.1, 0.4, 1.0};
    // ctrl = {0.75, 1.0, 0.8, 0.8, 0.1, 0.95, 0.7, 0.2, 0.35, 0.85, 0.85, 0.95, 0.7, 1.0, 0.8, 0.7, 0.2, 1.0, 1.0, 0.7, 0.45, 0.15, 0.15, 0.75, 1.0, 0.15, 0.5, 0.1, 0.5, 0.85, 0.25, 0.1, 0.9, 0.2, 0.9, 0.9};
    simu.controller().set_controller(ctrl);
    simu.run(3);
    std::cout<<simu.covered_distance()<<" "<<simu.arrival_angle()<<std::endl;
    global_robot.reset();
    global_env.reset();
    dCloseODE();
	return 0;
}