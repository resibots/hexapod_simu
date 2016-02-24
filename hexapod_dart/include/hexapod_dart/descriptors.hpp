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

        ////////////////////////////////////////////////////////////////////////

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

        ////////////////////////////////////////////////////////////////////////

        struct PositionTraj : public DescriptorBase {
        public:
            PositionTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                Eigen::Matrix3d rr = dart::math::expMapRot(rob->rot());
                Eigen::Matrix3d ro = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                Eigen::MatrixXd init(4, 4);
                Eigen::Vector6d pose = rob->pose();
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

        ////////////////////////////////////////////////////////////////////////

        struct PositionDiffTraj : public DescriptorBase {
        public:
            PositionDiffTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
              if (_pos_traj.size() == 0)
                _prev_trans = init_trans;

              Eigen::Matrix3d rr = dart::math::expMapRot(rob->rot());
              Eigen::Matrix3d ro = dart::math::expMapRot({_prev_trans[0], _prev_trans[1], _prev_trans[2]});
              Eigen::MatrixXd init(4, 4);
              Eigen::Vector6d pose = rob->pose();
              init << ro(0, 0), ro(0, 1), ro(0, 2), _prev_trans[3], ro(1, 0), ro(1, 1), ro(1, 2), _prev_trans[4], ro(2, 0), ro(2, 1), ro(2, 2), _prev_trans[5], 0, 0, 0, 1;
              Eigen::MatrixXd pp(4, 4);
              pp << rr(0, 0), rr(0, 1), rr(0, 2), pose[3], rr(1, 0), rr(1, 1), rr(1, 2), pose[4], rr(2, 0), rr(2, 1), rr(2, 2), pose[5], 0, 0, 0, 1;
              Eigen::Vector4d p = {_prev_trans[3], _prev_trans[4], _prev_trans[5], 1.0};
              p = init.inverse() * pp * p;

              _pos_traj.push_back({p[0], p[1], p[2]});

              // the previous transformation becomes the current pose
              _prev_trans = pose;
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _pos_traj;
            }

        protected:
            Eigen::Vector6d _prev_trans;
            std::vector<Eigen::Vector3d> _pos_traj;
        };

        ////////////////////////////////////////////////////////////////////////

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

                _rotation_traj.push_back(rpy);
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _rotation_traj;
            }

        protected:
            std::vector<Eigen::Vector3d> _rotation_traj;
        };

        ////////////////////////////////////////////////////////////////////////

        struct RotationDiffTraj : public DescriptorBase {
        public:
            RotationDiffTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
              if (_rotation_traj.size() == 0)
                _prev_trans = init_trans;

              // roll-pitch-yaw
              Eigen::Matrix3d rr = dart::math::expMapRot(rob->rot());
              Eigen::Matrix3d ro = dart::math::expMapRot({_prev_trans[0], _prev_trans[1], _prev_trans[2]});
              Eigen::Vector3d rpy = dart::math::matrixToEulerXYZ(ro.inverse() * rr);

              _rotation_traj.push_back(rpy);

              _prev_trans = rob->pose();
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _rotation_traj;
            }

        protected:
            Eigen::Vector6d _prev_trans;
            std::vector<Eigen::Vector3d> _rotation_traj;
        };

        ////////////////////////////////////////////////////////////////////////

        struct JointsTraj : public DescriptorBase {
        public:
            JointsTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                std::vector<size_t> ind;
                for(size_t i=0;i<rob->skeleton()->getNumDofs()-6;i++)
                  ind.push_back(i+6);
                Eigen::VectorXd pp = rob->skeleton()->getPositions(ind);
                std::vector<double> pos;
                for(size_t i=0;i<pp.size();i++)
                  pos.push_back(pp(i));
                _jnt_traj.push_back(pos);
            }

            void get(std::vector<std::vector<double>>& results)
            {
                results = _jnt_traj;
            }

        protected:
            std::vector<std::vector<double>> _jnt_traj;
        };
    }
}

#endif
