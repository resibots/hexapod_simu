#ifndef HEXAPOD_DART_DESCRIPTORS_HPP
#define HEXAPOD_DART_DESCRIPTORS_HPP

#include <map>
#include <vector>
#include <numeric>

#include <Eigen/Core>

#include <hexapod_dart/hexapod.hpp>

namespace hexapod_dart {

    namespace descriptors {

        struct DescriptorBase {
        public:
            using robot_t = std::shared_ptr<Hexapod>;

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_rot)
            {
                assert(false);
            }

            template <typename T>
            void get(T& results)
            {
                assert(false);
            }
        };

        struct DutyCycle : public DescriptorBase {
        public:
            DutyCycle()
            {
                for (size_t i = 0; i < 6; i++)
                    _contacts[i] = std::vector<size_t>();
            }

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_rot)
            {
                for (size_t i = 0; i < 6; ++i) {
                    std::string leg_name = "leg_" + std::to_string(i) + "_3";
                    dart::dynamics::BodyNodePtr body_to_check;
                    // TO-DO: Maybe there's a cleaner way to get the body
                    for (size_t j = 0; j < rob->skeleton()->getNumBodyNodes(); j++) {
                        auto bd = rob->skeleton()->getBodyNode(j);
                        if (leg_name == bd->getName())
                            body_to_check = bd;
                    }
                    if (rob->is_broken(i)) {
                        _contacts[i].push_back(0);
                    }
                    else {
                        _contacts[i].push_back(body_to_check->isColliding());
                    }
                }
            }

            void get(std::vector<double>& results)
            {
                for (size_t i = 0; i < 6; i++) {
                    results.push_back(std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0);
                }
            }

        protected:
            std::map<size_t, std::vector<size_t>> _contacts;
        };

        struct PositionTraj : public DescriptorBase {
        public:
            PositionTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_rot)
            {
                _pos_traj.push_back(rob->pos() - init_pos);
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _pos_traj;
            }

        protected:
            std::vector<Eigen::Vector3d> _pos_traj;
        };

        struct RotationTraj : public DescriptorBase {
        public:
            RotationTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_rot)
            {
                // roll-pitch-yaw
                auto rot_mat = dart::math::expMapRot(rob->rot() - init_rot);
                auto rpy = dart::math::matrixToEulerXYZ(rot_mat);

                _rotation_traj.push_back(std::round(rpy(2) * 100) / 100.0);
            }

            void get(std::vector<double>& results)
            {
                results = _rotation_traj;
            }

        protected:
            std::vector<double> _rotation_traj;
        };
    }
}

#endif
