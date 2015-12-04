#include <hexapod_dart_simu.hpp>


HexapodDARTSimu::HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) :
	_controller(ctrl, robot->broken_legs()),
	_covered_distance(0.0),
	_energy(0.0),
	_world(std::make_shared<dart::simulation::World>()),
	_old_t(0.0),
	_old_index(0),
	_init(false)
{
	// TO-DO Initialization of world/robot
	_robot = robot;
}