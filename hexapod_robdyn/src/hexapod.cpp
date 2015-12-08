#include <hexapod.hpp>
#include <ode/box.hh>
#include <ode/capped_cyl.hh>
#include <ode/sphere.hh>
#include <ode/motor.hh>
#include <ode/mx28.hh>
#include <ode/ax12.hh>

namespace robot {

    Hexapod::Hexapod(ode::Environment_hexa& env, const Eigen::Vector3d& pos, std::vector<int> broken_legs) : _broken_legs(broken_legs)
    {
        _build(env, pos);
    }

    Hexapod::Hexapod(const Hexapod& o, ode::Environment_hexa& env) : Robot(o, env)
    {
        for (size_t i = 1; i < _bodies.size(); i++) {
            env.add_leg_object((i - 1) / 3, *_bodies[i]);
        }

        _broken_legs = o._broken_legs;
    }

    boost::shared_ptr<Hexapod> Hexapod::clone(ode::Environment_hexa& env) const
    {
        return boost::shared_ptr<Hexapod>(new Hexapod(*this, env));
    }

    void Hexapod::move_joints(const std::vector<double>& angles)
    {
        assert(angles.size() >= this->servos().size());

        for (size_t i = 0; i < this->servos().size(); i++){
            this->servos()[i]->set_angle(0, angles[i]);
        }
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

    void Hexapod::_build(ode::Environment_hexa& env, const Eigen::Vector3d& pos)
    {

        /// Definition of robot's params
        // length in meter
        // mass in KG
        /*      static const double body_mass = 1.6; // with mx 28
        static const double body_length = 0.20;
        static const double body_width = 0.24;
        static const double body_height = 0.04;

        static const double legP1_w = 0.02;
        static const double legP1_length = 0.06;
	//        static const double legP1_dist = 0.2;
        static const double legP1_mass = 0.020;

        static const double legP2_w = 0.02;
        static const double legP2_length = 0.085;
	//        static const double legP2_dist = 0.2;
        static const double legP2_mass = 0.180;

        static const double legP3_w = 0.025;
        static const double legP3_length = 0.145;
	//        static const double legP3_dist = 0.2;
        static const double legP3_mass = 0.060;
      */

        static const double battery = 0.292; // white LiPo
        static const double computer = 0.277; // cf documentation
        static const double body_mass = 1.6 - battery - computer; // with the new Mx 28
        static const double body_length = 0.20;
        static const double body_width = 0.24;
        static const double body_height = 0.04;

        static const double legP1_w = 0.02;
        static const double legP1_length = 0.06;
        // static const double legP1_dist = 0.2;
        static const double legP1_mass = 0.020;

        static const double legP2_w = 0.02;
        static const double legP2_length = 0.085;
        // static const double legP2_dist = 0.2;
        static const double legP2_mass = 0.184;

        static const double legP3_w = 0.025;
        static const double legP3_length = 0.095;
        // static const double legP3_dist = 0.2;
        static const double legP3_mass = 0.040;

        /// creation of robot's body
        _main_body = ode::Object::ptr_t(new ode::Box(env, pos + Eigen::Vector3d(0, 0, legP3_length + 0.01),
            body_mass, body_length, body_width, body_height));
        _bodies.push_back(_main_body);

        for (int i = 0; i < 6; ++i) // for each legs
        {
            for (size_t j = 0; j < _broken_legs.size(); j++) {
                if (i == _broken_legs[j]) {
                    i++;
                    if (_broken_legs.size() > j + 1 && _broken_legs[j + 1] != i)
                        break;
                }
            }
            if (i >= 6)
                return;
            // selecting an angle corresponding to number of the leg
            //             float angle = i<3 ? M_PI_2.0f : -M_PI_2.0f;// + M_PI / 6;
            float angle = 0;
            float xStart = 0; // selection of the start of the first joint
            float yStart = 0;
            switch (i) {
            case 0:
            case 5: {
                xStart = i < 3 ? 0.06 : -0.06;
                yStart = 0.12;
                angle = i < 3 ? 3 * M_PI_4 / 2 : -3 * M_PI_4 / 2;
                break;
            }

            case 1:
            case 4: {
                xStart = i < 3 ? 0.10 : -0.10;
                yStart = 0;
                angle = i < 3 ? M_PI_2 : -M_PI_2;
                break;
            }

            case 2:
            case 3: {
                xStart = i < 3 ? 0.06 : -0.06;
                yStart = -0.12;
                angle = i < 3 ? 5 * M_PI_4 / 2 : -5 * M_PI_4 / 2;
                break;
            }
            }

            /// first part
            ode::Object::ptr_t l1(
                new ode::CappedCyl(env, pos
                        + Eigen::Vector3d(xStart + sin(angle) * (legP1_length / 2),
                                       yStart + cos(angle) * (legP1_length / 2),
                                       legP3_length + 0.01),
                    legP1_mass, legP1_w, legP1_length));

            l1->set_rotation(Eigen::Vector3d(cos(-angle), sin(-angle), 0),
                Eigen::Vector3d(0, 0, -1));

            _bodies.push_back(l1);
            env.add_leg_object(i, *l1);
            ode::Mx28::ptr_t s1(new ode::Mx28(env, pos + Eigen::Vector3d(xStart,
                                                   yStart,
                                                   /*legP2_length+*/ legP3_length + 0.01),
                *_main_body, *l1));

            //s1->set_axis(0, Eigen::Vector3d(cos(-angle), sin(-angle), 0));
            s1->set_axis(0, Eigen::Vector3d(0, 0, 1));
            //s1->set_axis(2, Eigen::Vector3d(0, 0, -1));

            s1->set_lim(0, -M_PI_4 / 2, M_PI_4 / 2);

            _servos.push_back(s1);

            /// second part
            ode::Object::ptr_t l2(
                new ode::CappedCyl(env, pos + Eigen::Vector3d(xStart + sin(angle) * (legP1_length + legP2_length / 2),
                                             yStart + cos(angle) * (legP1_length + legP2_length / 2),
                                             legP3_length + 0.01),
                    legP2_mass, legP2_w, legP2_length));

            /*l2->set_rotation(Eigen::Vector3d(cos(-angle), sin(-angle), 0),
                             Eigen::Vector3d(sin(-angle), -cos(-angle), 0));*/
            l2->set_rotation(Eigen::Vector3d(cos(-angle), sin(-angle), 0),
                Eigen::Vector3d(0, 0, -1));
            _bodies.push_back(l2);

            env.add_leg_object(i, *l2);

            ode::Mx28::ptr_t s2(new ode::Mx28(env, pos + Eigen::Vector3d(xStart + sin(angle) * (legP1_length),
                                                   yStart + cos(angle) * (legP1_length),
                                                   legP3_length + 0.01),
                *l1, *l2));
            s2->set_axis(0, Eigen::Vector3d(cos(-angle), sin(-angle), 0));
            s2->set_axis(2, Eigen::Vector3d(sin(-angle), -cos(-angle), 0));
            s2->set_lim(0, -M_PI_4, M_PI_4);
            _servos.push_back(s2);

            /// third part
            ode::Object::ptr_t l3(
                new ode::CappedCyl(env, pos + Eigen::Vector3d(xStart + sin(angle) * (legP1_length + legP2_length),
                                             yStart + cos(angle) * (legP1_length + legP2_length),
                                             legP3_length / 2 + 0.01),
                    legP3_mass, legP3_w, legP3_length));

            l3->set_rotation(Eigen::Vector3d(cos(-angle), sin(-angle), 0),
                Eigen::Vector3d(sin(-angle), -cos(-angle), 0));

            _bodies.push_back(l3);
            env.add_leg_object(i, *l3);
            ode::Mx28::ptr_t s3(new ode::Mx28(env, pos + Eigen::Vector3d(xStart + sin(angle) * (legP1_length + legP2_length),
                                                   yStart + cos(angle) * (legP1_length + legP2_length),
                                                   legP3_length + 0.01),
                *l2, *l3));
            s3->set_axis(0, Eigen::Vector3d(cos(-angle), sin(-angle), 0));
            s3->set_axis(2, Eigen::Vector3d(sin(-angle), -cos(-angle), 0));

            s3->set_lim(0, -M_PI_4, M_PI_4);
            _servos.push_back(s3);
        }

        for (size_t i = 0; i < _servos.size(); ++i)
            for (size_t j = 0; j < 3; ++j)
                _servos[i]->set_lim(j, -M_PI_2, M_PI_2);
    }
}