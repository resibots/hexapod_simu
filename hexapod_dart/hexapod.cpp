#include <hexapod.hpp>

namespace robot {
	Hexapod::Hexapod(std::string urdf_file, std::vector<int> broken_legs) : _broken_legs(broken_legs)
	{
		assert(_load_urdf(urdf_file));
		// TO-DO: remove broken legs from skeleton
	}

	Hexapod::Hexapod(dart::dynamics::SkeletonPtr skeleton, std::vector<int> broken_legs) :
		_skeleton(skeleton),
		_broken_legs(broken_legs)
	{
		// TO-DO: remove broken legs from skeleton
	}

	std::shared_ptr<Hexapod> Hexapod::clone() const
	{
		return std::make_shared<Hexapod>(_skeleton->clone(), _broken_legs);
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
		// Load the Skeleton from a file
		dart::utils::DartLoader loader;
		_skeleton = loader.parseSkeleton(urdf_file);
		if (_skeleton == nullptr)
			return false;
		_skeleton->setName("hexapod");

		// Set joint limits
		for(size_t i = 0; i < _skeleton->getNumJoints(); ++i)
			_skeleton->getJoint(i)->setPositionLimitEnforced(true);

		// _skeleton->setPosition(5, 0.1);
		return true;
    }

}