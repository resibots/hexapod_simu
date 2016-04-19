#ifndef HEXAPOD_DART_HEXAPOD_HPP
#define HEXAPOD_DART_HEXAPOD_HPP

#include <dart/dart.h>
#include <dart/utils/urdf/urdf.h>
#include <Eigen/Core>
#include <string>
#include <fstream>
#include <streambuf>

namespace hexapod_dart {
    class Hexapod {
    public:
        Hexapod() {}

        Hexapod(std::string urdf_file, std::vector<int> broken_legs) : _skeleton(_load_urdf(urdf_file))
        {
            assert(_skeleton != nullptr);
            _set_broken(broken_legs);
        }

        Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<int> broken_legs) : _skeleton(skeleton)
        {
            assert(_skeleton != nullptr);
            _set_broken(broken_legs);
        }

        std::shared_ptr<Hexapod> clone() const
        {
            // safely clone the skeleton
            _skeleton->getMutex().lock();
            auto tmp_skel = _skeleton->clone();
            _skeleton->getMutex().unlock();
            auto hexapod = std::make_shared<Hexapod>();
            hexapod->_skeleton = tmp_skel;
            hexapod->_broken_legs = _broken_legs;
            return hexapod;
        }

        dart::dynamics::SkeletonPtr skeleton()
        {
            return _skeleton;
        }

        bool is_broken(int leg)
        {
            for (size_t j = 0; j < _broken_legs.size(); j++) {
                if (leg == _broken_legs[j]) {
                    return true;
                }
            }
            return false;
        }

        std::vector<int> broken_legs()
        {
            return _broken_legs;
        }

        Eigen::Vector3d pos()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            return {pos_and_rot(3), pos_and_rot(4), pos_and_rot(5)};
        }

        Eigen::Vector3d rot()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            return {pos_and_rot(0), pos_and_rot(1), pos_and_rot(2)};
        }

        Eigen::Vector6d pose()
        {
            // DART's getPositions returns: COM orientation, COM position, joint positions
            auto pos_and_rot = _skeleton->getPositions();
            Eigen::Vector6d tmp;
            tmp << pos_and_rot(0), pos_and_rot(1), pos_and_rot(2), pos_and_rot(3), pos_and_rot(4), pos_and_rot(5);
            return tmp;
        }

    protected:
        dart::dynamics::SkeletonPtr _load_urdf(std::string urdf_file)
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
            return tmp_skel;
        }

        void _set_broken(const std::vector<int>& broken_legs)
        {
            _broken_legs = broken_legs;
            std::sort(_broken_legs.begin(), _broken_legs.end());
            _remove_legs();
        }

        void _remove_legs()
        {
            std::vector<int> to_remove;
            for (size_t i = 1; i < _skeleton->getNumJoints(); i += 3) {
                std::vector<int>::iterator it = std::find(_broken_legs.begin(), _broken_legs.end(), (i - 1) / 3);
                if (it != _broken_legs.end()) {
                    to_remove.push_back(i);
                }
            }

            for (size_t i = 0; i < to_remove.size(); i++) {
                auto tmp = _skeleton->getJoint(to_remove[i] - i * 3)->getChildBodyNode();
                tmp->removeAllShapeNodes();
                tmp->remove();
            }
        }

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<int> _broken_legs;
    };
}

#endif
