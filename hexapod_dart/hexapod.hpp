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

        void move_joints(const std::vector<double>& angles);

        bool is_broken(int leg);
        void set_broken(const std::vector<int>& broken_legs);
        std::vector<int> broken_legs();

    protected:
        bool _load_urdf(std::string urdf_file);

        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<int> _broken_legs;
    };
}

#endif