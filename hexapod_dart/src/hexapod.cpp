#include <hexapod.hpp>
#include <string>
#include <fstream>
#include <streambuf>

namespace robot {
    Hexapod::Hexapod(std::string urdf_file, std::vector<int> broken_legs) : _skeleton(_load_urdf(urdf_file))
    {
        assert(_skeleton != nullptr);
        _set_broken(broken_legs);
    }

    Hexapod::Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<int> broken_legs) : _skeleton(skeleton)
    {
        assert(_skeleton != nullptr);
        _set_broken(broken_legs);
    }

    std::shared_ptr<Hexapod> Hexapod::clone() const
    {
        return std::make_shared<Hexapod>(_skeleton->clone(), _broken_legs);
    }

    dart::dynamics::SkeletonPtr Hexapod::skeleton()
    {
        return _skeleton;
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

    std::vector<int> Hexapod::broken_legs()
    {
        return _broken_legs;
    }

    dart::dynamics::SkeletonPtr Hexapod::_load_urdf(std::string urdf_file)
    {
        // Load file into string
        std::ifstream t(urdf_file);
        std::string str((std::istreambuf_iterator<char>(t)),
            std::istreambuf_iterator<char>());
        // Load the Skeleton from a file
        dart::utils::DartLoader loader;
        dart::dynamics::SkeletonPtr tmp_skel = loader.parseSkeletonString(str, "");
        if (tmp_skel == nullptr)
            return nullptr;
        tmp_skel->setName("hexapod");

        // Set joint limits/actuator types
        for (size_t i = 1; i < tmp_skel->getNumJoints(); ++i) {
            tmp_skel->getJoint(i)->setPositionLimitEnforced(true);
            tmp_skel->getJoint(i)->setActuatorType(dart::dynamics::Joint::VELOCITY);
        }

        tmp_skel->setPosition(5, 0.1);
        tmp_skel->setPosition(2, DART_PI);
        return tmp_skel;
    }

    void Hexapod::_set_broken(const std::vector<int>& broken_legs)
    {
        _broken_legs = broken_legs;
        std::sort(_broken_legs.begin(), _broken_legs.end());
        _remove_legs();
    }

    void Hexapod::_remove_legs()
    {
        std::vector<int> to_remove;
        for (int i = 1; i < _skeleton->getNumJoints(); i += 3) {
            std::vector<int>::iterator it = std::find(_broken_legs.begin(), _broken_legs.end(), i / 3);
            if (it != _broken_legs.end()) {
                to_remove.push_back(i);
            }
        }

        for (int i = 0; i < to_remove.size(); i++) {
            auto tmp = _skeleton->getJoint(to_remove[i] - i * 3)->getChildBodyNode();
            tmp->removeAllCollisionShapes();
            tmp->removeAllVisualizationShapes();
            tmp->remove();
        }
    }
}