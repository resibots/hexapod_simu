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
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
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
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
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
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                Eigen::Vector6d pose = rob->pose();
                Eigen::Matrix3d rr = dart::math::expMapRot({pose[0], pose[1], pose[2]});
                Eigen::Matrix3d ro = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                Eigen::MatrixXd init(4, 4);
                init << ro(0, 0), ro(0, 1), ro(0, 2), init_trans[3], ro(1, 0), ro(1, 1), ro(1, 2), init_trans[4], ro(2, 0), ro(2, 1), ro(2, 2), init_trans[5], 0, 0, 0, 1;
                Eigen::MatrixXd pp(4, 4);
                pp << rr(0, 0), rr(0, 1), rr(0, 2), pose[3], rr(1, 0), rr(1, 1), rr(1, 2), pose[4], rr(2, 0), rr(2, 1), rr(2, 2), pose[5], 0, 0, 0, 1;
                Eigen::Vector4d p = {init_trans[3], init_trans[4], init_trans[5], 1.0};
                p = init.inverse() * pp * p;

                _pos_traj.push_back({p[0], p[1], p[2]});
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
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                // roll-pitch-yaw
                Eigen::Matrix3d rr = dart::math::expMapRot(rob->rot());
                Eigen::Matrix3d ro = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                auto rpy = dart::math::matrixToEulerXYZ(ro.inverse() * rr);

                _rotation_traj.push_back(rpy(2));
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
