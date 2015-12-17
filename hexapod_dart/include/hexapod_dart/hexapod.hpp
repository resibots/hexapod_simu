#ifndef HEXAPOD_DART_HEXAPOD_HPP
#define HEXAPOD_DART_HEXAPOD_HPP

#include <dart/dart.h>
#include <Eigen/Core>

namespace robot {
    class Hexapod {
    public:
        Hexapod(std::string urdf_file, std::vector<int> broken_legs);
        Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<int> broken_legs);

        std::shared_ptr<Hexapod> clone() const;

        dart::dynamics::SkeletonPtr skeleton();

        bool is_broken(int leg);
        std::vector<int> broken_legs();

        Eigen::Vector3d pos();
        Eigen::Vector3d rot();
        Eigen::Vector6d pose();

    protected:
        dart::dynamics::SkeletonPtr _load_urdf(std::string urdf_file);

        void _set_broken(const std::vector<int>& broken_legs);
        void _remove_legs();

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<int> _broken_legs;
    };
}

#endif