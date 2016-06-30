#ifndef HEXAPOD_DART_HEXAPOD_HPP
#define HEXAPOD_DART_HEXAPOD_HPP

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <Eigen/Core>
#include <string>
#include <fstream>
#include <streambuf>

namespace hexapod_dart {

    struct HexapodDamage {
        std::string type;
        std::string data;
        void* extra = nullptr;
    };

    class Hexapod {
    public:
        Hexapod() {}

        Hexapod(std::string urdf_file, std::vector<HexapodDamage> damages)
        {
            _damages = damages;
            _skeleton = _load_urdf(urdf_file);
            assert(_skeleton != nullptr);
            _set_damages(damages);
        }

        Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<HexapodDamage> damages) : _skeleton(skeleton)
        {
            assert(_skeleton != nullptr);
            _set_damages(damages);
        }

        std::shared_ptr<Hexapod> clone() const
        {
            // safely clone the skeleton
            _skeleton->getMutex().lock();
            auto tmp_skel = _skeleton->clone();
            _skeleton->getMutex().unlock();
            auto hexapod = std::make_shared<Hexapod>();
            hexapod->_skeleton = tmp_skel;
            hexapod->_damages = _damages;
            hexapod->_broken_legs = _broken_legs;
            return hexapod;
        }

        dart::dynamics::SkeletonPtr skeleton()
        {
            return _skeleton;
        }

        bool is_broken(int leg) const
        {
            for (size_t j = 0; j < _broken_legs.size(); j++) {
                if (leg == _broken_legs[j]) {
                    return true;
                }
            }
            return false;
        }

        std::vector<int> broken_legs() const
        {
            return _broken_legs;
        }

        std::vector<HexapodDamage> damages() const
        {
            return _damages;
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

            // TO-DO: Do it in a proper way
            for (auto dmg : _damages) {
                if (dmg.type == "leg_shortening") {
                    std::string leg_link_name_start = "<link name=\"leg_" + std::string(1, dmg.data[0]) + "_3\">";
                    std::string leg_link_end = "</link>";
                    size_t found = str.find(leg_link_name_start);
                    if (found != std::string::npos) {
                        size_t found2 = str.find(leg_link_end, found);
                        std::string to_replace = "<visual><origin rpy=\"1.57079632679 0 0\" xyz=\"0.0125 0.035 0\"/><geometry><cylinder length=\"0.07\" radius=\"0.025\"/></geometry><material name=\"Red\"/></visual><visual><origin rpy=\"1.57079632679 0 0\" xyz=\"0.0125 0.0475 0\"/><geometry><sphere radius=\"0.025\"/></geometry><material name=\"Red\"/></visual><collision><origin rpy=\"1.57079632679 0 0\" xyz=\"0.0125 0.035 0\"/><geometry><box size=\"0.025 0.025 0.07\"/></geometry></collision><collision><origin rpy=\"1.57079632679 0 0\" xyz=\"0.0125 0.0475 0\"/><geometry><sphere radius=\"0.025\"/></geometry><material name=\"Red\"/></collision><inertial><!-- CENTER OF MASS --><origin rpy=\"1.57079632679 0 0\" xyz=\"0.0125 0.035 0\"/><mass value=\"0.04\"/><!-- box inertia: 1/12*m(y^2+z^2), ... --><inertia ixx=\"6.74166666667e-05\" ixy=\"0\" ixz=\"0\" iyy=\"6.74166666667e-05\" iyz=\"0\" izz=\"4.16666666667e-06\"/></inertial>";
                        str.replace(found + 22, found2 - found - 22, to_replace);
                    }
                }
            }
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

        void _set_damages(const std::vector<HexapodDamage>& damages)
        {
            _broken_legs.clear();

            _damages = damages;
            for (auto dmg : _damages) {
                if (dmg.type == "leg_removal") {
                    for (size_t i = 0; i < dmg.data.size(); i++) {
                        int l = dmg.data[i] - '0';
                        _broken_legs.push_back(l);
                        std::string leg_bd_name = "leg_" + std::string(1, dmg.data[i]) + "_1";
                        auto bd = _skeleton->getBodyNode(leg_bd_name);
                        bd->removeAllShapeNodes();
                        bd->remove();
                    }
                }
                else if (dmg.type == "blocked_joint") {
                    auto jnt = _skeleton->getJoint(dmg.data);
                    if (dmg.extra)
                        jnt->setPosition(0, *((double*)dmg.extra));
                    jnt->setActuatorType(dart::dynamics::Joint::LOCKED);
                }
                else if (dmg.type == "free_joint") {
                    _skeleton->getJoint(dmg.data)->setActuatorType(dart::dynamics::Joint::PASSIVE);
                }
            }

            std::sort(_broken_legs.begin(), _broken_legs.end());
        }

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<HexapodDamage> _damages;
        std::vector<int> _broken_legs;
        std::vector<int> _removed_joints;
    };
}

#endif
