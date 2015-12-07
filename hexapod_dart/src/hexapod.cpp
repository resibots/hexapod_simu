#include <hexapod.hpp>
#include <string>
#include <fstream>
#include <streambuf>

namespace robot {
    Hexapod::Hexapod(std::string urdf_file, std::vector<int> broken_legs) : _broken_legs(broken_legs)
    {
        assert(_load_urdf(urdf_file));
        // TO-DO: remove broken legs from skeleton
    }

    Hexapod::Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<int> broken_legs) : _skeleton(skeleton),
                                                                                           _broken_legs(broken_legs)
    {
        // TO-DO: remove broken legs from skeleton
    }

    std::shared_ptr<Hexapod> Hexapod::clone() const
    {
        return std::make_shared<Hexapod>(_skeleton->clone(), _broken_legs);
    }

    dart::dynamics::SkeletonPtr Hexapod::skeleton()
    {
        return _skeleton;
    }

    void Hexapod::move_joints(const std::vector<double>& angles)
    {
        // TO-DO: Send commands to skeleton joints
    }

    bool Hexapod::is_broken(int leg)
    {
        for (size_t j = 0; j < _broken_legs.size(); j++) {
            if (leg == _broken_legs[j]) {
                return true;
            }
        }
        return false;
    }

    void Hexapod::set_broken(const std::vector<int>& broken_legs)
    {
        _broken_legs = broken_legs;
    }

    std::vector<int> Hexapod::broken_legs()
    {
        return _broken_legs;
    }

    bool Hexapod::_load_urdf(std::string urdf_file)
    {
        // Load file into string
        std::ifstream t(urdf_file);
        std::string str((std::istreambuf_iterator<char>(t)),
            std::istreambuf_iterator<char>());
        // Load the Skeleton from a file
        dart::utils::DartLoader loader;
        _skeleton = loader.parseSkeletonString(str, "");
        if (_skeleton == nullptr)
            return false;
        _skeleton->setName("hexapod");

        // Set joint limits/actuator types
        for (size_t i = 1; i < _skeleton->getNumJoints(); ++i)
        {
            _skeleton->getJoint(i)->setPositionLimitEnforced(true);
            _skeleton->getJoint(i)->setActuatorType(dart::dynamics::Joint::VELOCITY);
        }

        _skeleton->setPosition(5, 0.1);
        return true;
    }
}