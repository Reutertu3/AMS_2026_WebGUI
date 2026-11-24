#pragma once
#ifndef INCLUDED_INTERPOLATOR_H
#define INCLUDED_INTERPOLATOR_H

#include <cmath>
#include <limits>
#include <vector>
#include <functional>

#include <stdio.h>

#include "mrinterpoltypes.h" // old interpolator types
#include "mrcpptypes.h"

namespace imsl {
  namespace interpolator {

    template<typename real_t>
    struct PathParameters {
        real_t v_max,                            // maximum velocity
          a,                                     // acceleration
          omega_max,                             // maximum angular velocity in theta
          alpha;                                 // angular acceleration in theta
        mrc_localization_modes localizationmode; // backward compatibility
        unsigned int waitTime;                   // time to wait in ms
    };

    // base class for all velocity profiles
    template<typename real_t>
    class VelocityProfile {
      public:
        // time to stretch (distance)
        virtual real_t t2s(real_t t, real_t *v) = 0;
        // calculate new velocity profile at time t (breaking/acceleration event) with new velocity v
        virtual void setNewSpeed(real_t t, real_t v) = 0;
        // calculate new velocity profile at t = 0, stetch s, velocity v_max and acceleration a
        virtual void setNewSpeedAndAccel(real_t t, real_t v, real_t a) = 0;
        // calculate new velocity profile at t = 0, stetch s, velocity v_max and acceleration a
        virtual void init(real_t s, real_t v_max, real_t a) = 0;

        // returns the total time for the movement
        virtual real_t getTg() = 0;
        virtual real_t getT12() = 0;
        virtual real_t getT13() = 0;
        virtual real_t getS12() = 0;
        virtual real_t getS13() = 0;
        virtual real_t getVb() = 0;
        virtual real_t getSb() = 0;
        virtual real_t getSg() = 0;

        virtual ~VelocityProfile() { }
    };

    // simple trapezoidal or triangular velocity profile
    template<typename real_t>
    class SimpleVelProfile : public VelocityProfile<real_t> {
      private:
        real_t tg, // overall time for the movement tg = tb + t12 + t23 + t34
          sg,      // overall path length (stretch) sg = sb + s12 + s23 + s34
          a,       // absolute value of acceleration (a > 0)
          a1,      // acceleration / deceleration in pase 1
          v_max,   // maximum velocity
          tb,      // time at breaking event
          sb,      // path length (stretch) at breaking event
          vb,      // velocity at breaking event
          t12,     // time for phase 1 acceleration / deceleration from vb to v2
          t13,     // time for phase 1 + phase 2: t13 = t12 + t23
          s12,     // stretch in t12
          s13,     // stretch in t13, s13 = s12 + s23
          v2;      // velocity after phase 1

      public:
        SimpleVelProfile()
        {
          tb = sb = vb = 0.0;
        }

        ~SimpleVelProfile() override { }

        real_t getTg() override
        {
          return tg;
        }

        real_t getT12() override
        {
          return t12;
        }

        real_t getT13() override
        {
          return t13;
        }

        real_t getS12() override
        {
          return s12;
        }

        real_t getS13() override
        {
          return s13;
        }

        real_t getVb() override
        {
          return vb;
        }

        real_t getSb() override
        {
          return sb;
        }

        real_t getSg() override
        {
          return sg;
        }

        // init velocity profile
        void init(real_t s, real_t v_max, real_t a) override
        {
          real_t t23, // time with constant v
            t34,      // time for breaking until v = 0
            s23,      // distance in t23
            s34,      // distance in t34
            sr;       // remaining distance sr = sg - sb
                      //        if (a <= 0.0) return -1;
                      //        if (v_max < 0.0) return -1;
          if (a <= 0.0)
            throw IMSLex(1, "SimpleVelProfile::init(): acceleration <= 0"); // TODO define error numbers
          /* v_max < 0 is just a flag for driving backwards. For the velocity profile we will work with the abs value */
          if (v_max < 0.0) {
            v_max = std::fabs(v_max);
          }

          this->a = a;
          this->v_max = v_max;

          // special case: null movement
          if (std::abs(s) < imsl::eps<real_t>) {
            t12 = t13 = tg = 0.0;
            sg = s12 = s13 = 0.0;
            v2 = v_max;
            a1 = a;
            return;
          }

          // error, negative distance
          //        if (s < 0.0) return -1;
          if (s < 0.0)
            throw IMSLex(1, "SimpleVelProfile::init(): distance < 0");

          sg = s;       // total distance (stretch) to travel
          sr = sg - sb; // remaining distance

          // special case: v = 0.0 stop on path, tg -> inf
          if (v_max == 0.0) {
            // break from tb to v = 0
            a1 = -a;
            t12 = vb / a;
            s12 = vb * t12 / 2.0;
            t23 = imsl::inf<real_t>;
            t13 = t12 + t23;
            tg = t13;
            s23 = sr - s12;
            s13 = sr;
            v2 = 0.0;
            return;
          }

          // calculation of trapezoid profile
          // vb velocity at breaking time tb
          if (v_max > vb)
            a1 = a; // increase velocity in phase 1
          else
            a1 = -a;               // decrease velocity in phase 1
          t12 = (v_max - vb) / a1; // time for acceleration / deceleration in phase 1
          // s12 = vb * t12 + a1 / 2.0 * t12 * t12;    // distance for acceleration / deceleration in phase 1
          s12 = (vb + v_max) / 2.0 * t12; // distance for acceleration / deceleration in phase 1
          t34 = v_max / a;                // time for deceleration
          s34 = v_max / 2.0 * t34;        // distance for deceleration
          s23 = sr - s12 - s34;           // distance for constant velocity

          if (s23 > 0) { // trapezoid ok
            // parameters for interpoplation
            v2 = v_max;            // maximum velocity in phase 2
            t13 = t12 + s23 / v2;  // time until decelaration in phase 3
            s13 = sr - s34;        // distance after t13
          } else {                 // triangle profile
            real_t te,             // time from vb to 0 in the last subphase
              se;                  // distance from vb to 0 in the last subphase
            te = vb / a;           // time to stop from breaking velocity vb
            se = vb * te / 2.0;    // a / 2.0 * te * te; // breaking distance
            s12 = (sr - se) / 2.0; // distance for acceleration from vb to v2 in phase 1
            s34 = s12 + se;        // distance for deceleration
            t34 = sqrt(2.0 * s34 / a);
            //          if (std::isnan(t34)) return -1; // negative sqrt
            if (std::isnan(t34)) {
              // log_message(log_error, "SimpleVelProfile::init(): negative sqrt, t34 = NAN: t34 = %lf, s34 = %lf, s = %lf, v_max = %lf, a = %lf", t34, s34, s, v_max, a);
              throw IMSLex(1, "SimpleVelProfile::init(): negative sqrt, t34 = NAN"); // negative sqrt
            }
            v2 = a * t34; // maximum speed at time t12
            // s12 = s34 - se;
            t12 = t34 - te; // time for acceleration in phase 1
            // not correct:
            // t12 = sqrt(2.0 * s12 / a + vb*vb);
            // v2 = a * t12 + vb;
            s23 = 0.0;      // no distance for traveling with constant speed
            t23 = 0.0;      // no time for traveling with constant speed
            t13 = t12;      // breaking time is t12
            s13 = s12;      // braking distance is at s12
            t34 = t12 + te; // time for deceleration until stop
          } // triangle

          tg = tb + t13 + t34; // total time for movement
          // DEBUG output
          // printf("SimpleVelProfile::init() tg: %2.3f, t12: %2.3f, t34: %2.3f, tb: %2.3f, vb: %2.3f, v2: %2.3f\n", tg, t12, t34, tb, vb, v2);
          // printf("SimpleVelProfile::init() sg: %2.3f, sr: %2.3f, sb: %2.3f, s12: %2.3f, s23: %2.3f, s34: %2.3f\n", sg, sr, sb, s12, s23, s34);
          return;
        } // end init

        real_t t2s(real_t t, real_t *v) override
        {
          t = t - tb;
          if (t < 0.0) { // negative time -> starting point
            *v = 0.0;    // *v = vb;
            return 0.0;
          }
          if (t < t12) { // acceleration / deceleration in phase 1
            real_t s = sb + vb * t + a1 / 2.0 * t * t;
            *v = a1 * t + vb; // store v directly before return
            return s;
          }
          if (t < t13) { // constant speed in phase 2
            t = t - t12;
            real_t s = sb + s12 + v2 * t;
            *v = v2; // store v directly before return
            return s;
          }
          if (t < tg - tb) { // deceleration in phase 3
            t = t - t13;
            real_t s = sb + s13 + v2 * t - a / 2.0 * t * t;
            *v = v2 - a * t; // store v directly before return
            return s;
          }

          *v = 0.0; // t > tg -> end point
          return sg;
        }

        // calculation of new velocity profile
        void setNewSpeed(real_t t, real_t v) override
        {
          setNewSpeedAndAccel(t, v, this->a);
        }

        void setNewSpeedAndAccel(real_t t, real_t v, real_t new_a) override
        {
          if (t > (tb + t13))
            return; // already in phase 3, stopping

          // printf("sb: %2.3f, vb: %2.3f, tb: %2.3f\n", sb, vb, tb);
          real_t v_temp;
          sb = t2s(t, &v_temp); // stretch sb and velocity vb at time t (breaking event)
          tb = t;
          vb = v_temp;
          a = new_a;
          // DEBUG
          // printf("sb: %2.3f, vb: %2.3f, tb: %2.3f\n", sb, vb, tb);
          init(sg, v, a); // calculate new profile
        }

    }; // class SimpleVelProfile

    // base class for all path segments
    template<typename real_t>
    class PathSegment {
      protected:
        imsl::PoseV<real_t> Pstart, // start pose of path segment
          Pend,                     // end pose of path segment
          Pact;                     // actual pose

        real_t tg; // whole time for movement
        PathParameters<real_t> para;
        VelocityProfile<real_t> *profile;

      public:
        struct
        {
            std::vector<std::function<int()>> pre;  // functors to be executed before driving the actual PathSegment
            std::vector<std::function<int()>> post; // functors to be executed after driving the actual PathSegment
        } functors;

        PathSegment() { }
        virtual ~PathSegment() { }

        PathSegment(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End)
          : Pstart(Start)
          , Pend(End)
          , Pact(Start)
          , tg(0.0)
        {
          // DEBUG output
          // printf("Pstart.x: %2.3lf; Pstart.y: %2.3lf; Pstart.theta: %2.3lf; Pend.x: %2.3lf; Pend.y:  %2.3lf; Pend.theta: %2.3lf\n",
          //        Pstart.x, Pstart.y, (real_t)Pstart.theta, Pend.x, Pend.y, (real_t)Pend.theta);
        }

        PathSegment(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, PathParameters<real_t> p)
          : Pstart(Start)
          , Pend(End)
          , Pact(Start)
          , tg(0.0)
          , para(p)
        {
          // DEBUG output
          // printf("Pstart.x: %2.3lf; Pstart.y: %2.3lf; Pstart.theta: %2.3lf; Pend.x: %2.3lf; Pend.y:  %2.3lf; Pend.theta: %2.3lf\n",
          //        Pstart.x, Pstart.y, (real_t)Pstart.theta, Pend.x, Pend.y, (real_t)Pend.theta);
        }

        // calcutaion of pose and velocity in a new time step of interpolation
        virtual imsl::PoseV<real_t> Interpolation(real_t t) = 0;
        virtual void setNewSpeed(real_t t, real_t v, real_t omega) = 0;
        virtual void setNewSpeedAndAccel(real_t t, real_t v, real_t omega, real_t a, real_t alpha) = 0;

        real_t getTg()
        {
          return tg;
        }

        imsl::PoseV<real_t> getStart()
        {
          return Pstart;
        }

        imsl::PoseV<real_t> getEnd()
        {
          return Pend;
        }

        VelocityProfile<real_t> *getProfile()
        {
          return profile;
        }

        PathParameters<real_t> getPathParameters()
        {
          return para;
        };

    }; // class PathSegment

    // path segment for linear translation without rotation
    template<typename real_t>
    class LinearTranslation : public PathSegment<real_t> {
      protected:
        imsl::Heading<real_t> alpha; // constant heading on the path
        real_t sa;                   // sin(alpha)
        real_t ca;                   // cos(alpha)
        real_t pathlength;

        using PathSegment<real_t>::Pstart;
        using PathSegment<real_t>::Pend;
        using PathSegment<real_t>::Pact;
        using PathSegment<real_t>::profile;
        using PathSegment<real_t>::tg;
        using PathSegment<real_t>::para;

      public:
        LinearTranslation(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, real_t v_max, real_t a, real_t omega_max = 0.0, real_t Paramalpha = 0.0, mrc_localization_modes localizationmode = MRC_LOCALIZATION_OFF, unsigned int waitTime = 0)
          : PathSegment<real_t>(Start, End, {v_max, a, omega_max, Paramalpha, localizationmode, waitTime})
        {
          real_t dx, dy;
          dx = (Pend.x - Pstart.x);
          dy = (Pend.y - Pstart.y);
          alpha = std::atan2(dy, dx);
          para.v_max = v_max;
          ca = std::cos(alpha);
          sa = std::sin(alpha);
          if (v_max < 0) {
            v_max = std::fabs(v_max);
            alpha += M_PI;
          }
          pathlength = std::sqrt(dx * dx + dy * dy);
          profile = new SimpleVelProfile<real_t>();
          profile->init(pathlength, v_max, a);
          tg = profile->getTg();
          para.a = a;
          // DEBUG output
          // printf("LinearTranslation::LinearTranslation() dx: %2.3f, dy: %2.3f, alpha: %2.3f, ca: %2.3f, sa: %2.3f, pathlength: %2.3f\n",
          //        dx, dy, (real_t)alpha, ca, sa, pathlength);
        }

        LinearTranslation(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, PathParameters<real_t> p)
          : LinearTranslation(Start, End, p.vmax, p.a, p.omega_max, p.alpha, p.localizationmode, p.waitTime)
        {
        }

        ~LinearTranslation() override
        {
          delete (profile);
        }

        real_t getPathlength()
        {
          return pathlength;
        }

        imsl::Heading<real_t> getPathangle()
        {
          return alpha;
        }

        imsl::PoseV<real_t> Interpolation(real_t t) override
        {
          real_t v, ds;
          ds = profile->t2s(t, &v);

          Pact.x = Pstart.x + ds * ca;
          Pact.y = Pstart.y + ds * sa;
          Pact.theta = alpha;
          Pact.vx = v * ca;
          Pact.vy = v * sa;
          Pact.omega = 0.0;
          // DEBUG output
          // printf("LinearTranslation::Interpolation(%2.3f) ds: %2.3f, x: %2.3f, y: %2.3f, theta: %2.3f\n",
          //        t, ds, Pact.x, Pact.y, (real_t)Pact.theta);
          return Pact;
        }

        void setNewSpeed(real_t t, real_t v, real_t omega) override
        {
          profile->setNewSpeed(t, v);
          tg = profile->getTg();
        }

        void setNewSpeedAndAccel(real_t t, real_t v, real_t omega, real_t a, real_t alpha) override
        {
          profile->setNewSpeedAndAccel(t, v, a);
          tg = profile->getTg();
        }

    }; // class LinearTranslation

    // path segment for rotation on a spot, no translation
    template<typename real_t>
    class Rotation : public PathSegment<real_t> {
      protected:
        real_t angle;  // rotation angle
        int sgn_angle; // direction of rotation

        using PathSegment<real_t>::Pstart;
        using PathSegment<real_t>::Pend;
        using PathSegment<real_t>::Pact;
        using PathSegment<real_t>::profile;
        using PathSegment<real_t>::tg;
        using PathSegment<real_t>::para;

      public:
        Rotation(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, real_t omega_max, real_t alpha, real_t v_max = 0.0, real_t a = 0.0, mrc_localization_modes localizationmode = MRC_LOCALIZATION_OFF, unsigned int waitTime = 0)
          : PathSegment<real_t>(Start, End, {v_max, a, omega_max, alpha, localizationmode, waitTime})
        {
          angle = imsl::Heading<real_t>(End.theta - Start.theta);
          sgn_angle = imsl::signum<real_t>(angle);
          Pend.x = Pstart.x;
          Pend.y = Pstart.y;
          profile = new SimpleVelProfile<real_t>();
          profile->init(angle * sgn_angle, omega_max, alpha);
          tg = profile->getTg();
          para.v_max = 0.0;
          para.a = 0.0;
          para.omega_max = omega_max;
          para.alpha = alpha;
          // DEBUG output
          // printf("Rotation::Rotation() angle: %2.3f, sgn_angle: %i\n", angle, sgn_angle);
        }

        Rotation(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, PathParameters<real_t> p)
          : Rotation(Start, End, p.omega_max, p.alpha, p.v_max, p.a, p.localizationmode, p.waitTime)
        {
        }

        ~Rotation() override
        {
          delete (profile);
        }

        Rotation(imsl::PoseV<real_t> start, real_t ang, real_t omega_max, real_t alpha)
          : PathSegment<real_t>(start, start)
        {
          angle = ang;
          sgn_angle = imsl::signum<real_t>(angle);
          Pend.theta = Pstart.theta + angle;
          profile = new SimpleVelProfile<real_t>();
          profile->init(angle * sgn_angle, omega_max, alpha);
          tg = profile->getTg();
          para.v_max = 0.0;
          para.a = 0.0;
          para.omega_max = omega_max;
          para.alpha = alpha;
        }

        real_t getAngle()
        {
          return angle;
        } // rotation angle

        int getSgnAngle()
        {
          return sgn_angle;
        } // direction of rotation

        imsl::PoseV<real_t> Interpolation(real_t t) override
        {
          real_t omega, phi;
          phi = profile->t2s(t, &omega);

          Pact.x = Pstart.x;
          Pact.y = Pstart.y;
          Pact.theta = Pstart.theta + phi * sgn_angle;
          Pact.vx = 0.0;
          Pact.vy = 0.0;
          Pact.omega = omega * sgn_angle;
          return Pact;
        }

        void setNewSpeed(real_t t, real_t v, real_t omega) override
        {
          profile->setNewSpeed(t, omega);
          tg = profile->getTg();
        }

        void setNewSpeedAndAccel(real_t t, real_t v, real_t omega, real_t a, real_t alpha) override
        {
          profile->setNewSpeedAndAccel(t, omega, alpha);
          tg = profile->getTg();
        }
    }; // end class Rotation

    // path segment for combined rotation and translation, rot -> trans -> rot
    template<typename real_t>
    class RotTransRot : public PathSegment<real_t> {
      protected:
        Rotation<real_t> *rot1, *rot2;
        LinearTranslation<real_t> *trans;
        real_t t_rot1, t_trans, t_rot2;

        using PathSegment<real_t>::Pstart;
        using PathSegment<real_t>::Pend;
        using PathSegment<real_t>::Pact;
        using PathSegment<real_t>::profile;
        using PathSegment<real_t>::tg;
        using PathSegment<real_t>::para;

      public:
        RotTransRot(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, real_t v_max, real_t a, real_t omega_max, real_t alpha, mrc_localization_modes localizationmode = MRC_LOCALIZATION_OFF, unsigned int waitTime = 0)
          : PathSegment<real_t>(Start, End, {v_max, a, omega_max, alpha, localizationmode, waitTime})
        {
          trans = new LinearTranslation<real_t>(Start, End, v_max, a);
          real_t path_angle = trans->getPathangle();
          t_trans = trans->getTg();
          real_t path_length = trans->getPathlength();

          para.v_max = v_max;
          para.a = a;
          para.omega_max = omega_max;
          para.alpha = alpha;

          if (std::abs(path_length) < imsl::eps<real_t>) {
            t_trans = 0.0;
            rot1 = new Rotation<real_t>(Start, End, omega_max, alpha);
            t_rot1 = rot1->getTg();
            rot2 = new Rotation<real_t>(End, End, omega_max, alpha);
            t_rot2 = 0.0;
          } else {
            imsl::Heading<real_t> rot_angle = path_angle - Start.theta;
            rot1 = new Rotation<real_t>(Start, rot_angle, omega_max, alpha);
            t_rot1 = rot1->getTg();

            rot_angle = End.theta - path_angle;
            imsl::PoseV<real_t> TransEndPose = End;
            TransEndPose.theta -= rot_angle;
            rot2 = new Rotation<real_t>(TransEndPose, rot_angle, omega_max, alpha);
            t_rot2 = rot2->getTg();
          }

          tg = t_rot1 + t_trans + t_rot2;
          // printf("RotTransRot::RotTransRot: tg = %2.2f; t_rot1 = %2.2f; t_trans = %2.2f; t_rot2 = %2.2f\n",
          //     tg, t_rot1, t_trans, t_rot2);
          // fflush(stdout);
        }

        RotTransRot(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, PathParameters<real_t> p)
          : RotTransRot(Start, End, p.v_max, p.a, p.omega_max, p.alpha, p.localizationmode, p.waitTime)
        {
        }

        ~RotTransRot()
        {
          delete (rot1);
          delete (trans);
          delete (rot2);
        }

        imsl::PoseV<real_t> Interpolation(real_t t) override
        {
          // real_t omega, phi;
          if (t < 0.0) {
            Pact = Pstart;
          } else if (t < t_rot1) {
            Pact = rot1->Interpolation(t);
          } else if (t < t_rot1 + t_trans) {
            Pact = trans->Interpolation(t - t_rot1);
          } else if (t < t_rot1 + t_trans + t_rot2) {
            Pact = rot2->Interpolation(t - t_rot1 - t_trans);
          } else {
            Pact = Pend;
          }
          return Pact;
        }

        void setNewSpeed(real_t t, real_t v, real_t omega) override
        {
          //        if (t < 0.0) return -1;
          if (t < 0.0)
            throw IMSLex(1, "RotTransRot::setNewSpeed(): t <0");
          /*
                  if (t < t_rot1) {
                    rot1->setNewSpeed(t, v, omega);
                    t_rot1 = rot1->getTg();
                  }
                  if (t < t_rot1 + t_trans &&
                      !(t_rot1 >= std::numeric_limits<real_t>::infinity())) {
                    trans->setNewSpeed(t - t_rot1, v, omega);
                    t_trans = trans->getTg();
                  }
                  if (t < t_rot1 + t_trans + t_rot2 &&
                      !(t_rot1 >= std::numeric_limits<real_t>::infinity()) &&
                      !(t_trans >= std::numeric_limits<real_t>::infinity())) {
                    rot2->setNewSpeed(t - t_rot1 - t_trans, v, omega);
                    t_rot2 = rot2->getTg();
                  }
          */
          // recalculate only the actual segment and segments in the future
          // tg (t_rot1,2, t_trans) of segments in the past is never infinity
          if (t < t_rot1) { // actual in segment rot1
            rot1->setNewSpeed(t, v, omega);
            t_rot1 = rot1->getTg();
            trans->setNewSpeed(0, v, omega); // set new speed at beginning of segment trans
            t_trans = trans->getTg();
            rot2->setNewSpeed(0, v, omega); // set new speed at beginning of segment rot2
            t_rot2 = rot2->getTg();
          } else if (t < t_rot1 + t_trans) { // actual in segment trans
            trans->setNewSpeed(t - t_rot1, v, omega);
            t_trans = trans->getTg();
            rot2->setNewSpeed(0, v, omega); // set new speed at beginning of segment rot2
            t_rot2 = rot2->getTg();
          } else if (t < t_rot1 + t_trans + t_rot2) { // actual in segment rot2
            rot2->setNewSpeed(t - t_rot1 - t_trans, v, omega);
            t_rot2 = rot2->getTg();
          }

          tg = t_rot1 + t_trans + t_rot2;
          return;
        }
        void setNewSpeedAndAccel(real_t t, real_t v, real_t omega, real_t a, real_t alpha) override
        {
          if (t < 0.0)
            throw IMSLex(1, "RotTransRot::setNewSpeed(): t <0");
          // recalculate only the actual segment and segments in the future
          // tg (t_rot1,2, t_trans) of segments in the past is never infinity
          if (t < t_rot1) { // actual in segment rot1
            rot1->setNewSpeedAndAccel(t, v, omega, a, alpha);
            t_rot1 = rot1->getTg();
            trans->setNewSpeedAndAccel(0, v, omega, a, alpha); // set new speed at beginning of segment trans
            t_trans = trans->getTg();
            rot2->setNewSpeedAndAccel(0, v, omega, a, alpha); // set new speed at beginning of segment rot2
            t_rot2 = rot2->getTg();
          } else if (t < t_rot1 + t_trans) { // actual in segment trans
            trans->setNewSpeedAndAccel(t - t_rot1, v, omega, a, alpha);
            t_trans = trans->getTg();
            rot2->setNewSpeedAndAccel(0, v, omega, a, alpha); // set new speed at beginning of segment rot2
            t_rot2 = rot2->getTg();
          } else if (t < t_rot1 + t_trans + t_rot2) { // actual in segment rot2
            rot2->setNewSpeedAndAccel(t - t_rot1 - t_trans, v, omega, a, alpha);
            t_rot2 = rot2->getTg();
          }

          tg = t_rot1 + t_trans + t_rot2;
          return;
        }

    }; // end class RotTransRot

    // Path segment for rotation and translation at a same time
    template<typename real_t>
    class RotAndTrans : public PathSegment<real_t> {
      protected:
        LinearTranslation<real_t> *trans;
        Rotation<real_t> *rot;
        real_t get_angle;
        int sgn_angle; // direction of rotation
        imsl::Heading<real_t> path_angle;
        real_t path_length;
        imsl::PoseV<real_t> P_temp;

        using PathSegment<real_t>::Pstart;
        using PathSegment<real_t>::Pend;
        using PathSegment<real_t>::Pact;
        using PathSegment<real_t>::profile;
        using PathSegment<real_t>::tg;
        using PathSegment<real_t>::para;

      public:
        RotAndTrans(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, real_t v_max, real_t a, real_t omega_max, real_t alpha, mrc_localization_modes localizationmode = MRC_LOCALIZATION_OFF, unsigned int waitTime = 0)
          : PathSegment<real_t>(Start, End, {v_max, a, omega_max, alpha, localizationmode, waitTime})
        {

          real_t tg_rot,
            tg_trans,
            t12,
            t13;

          trans = new LinearTranslation<real_t>(Start, End, v_max, a);
          path_angle = trans->getPathangle();
          path_length = trans->getPathlength();
          tg_trans = trans->getTg();

          rot = new Rotation<real_t>(Start, End, omega_max, alpha);
          get_angle = Start.theta - End.theta; // rotation of angle
          sgn_angle = imsl::signum<real_t>(get_angle);
          tg_rot = rot->getTg();

          if (tg_trans != 0.0 && tg_rot != 0.0) //
          {
            if (tg_trans < tg_rot) {
              tg = tg_rot;
              t12 = rot->getProfile()->getT12();
              t13 = rot->getProfile()->getT13();

              real_t new_v_max = (path_length - 0.5 * trans->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t13) + 0.5 * t12);
              real_t new_a = (new_v_max - trans->getProfile()->getVb()) / t12;

              trans->setNewSpeedAndAccel(0.0, new_v_max, 0.0, new_a, 0.0);
              // trans->getProfile()->init(path_length, new_v_max, new_a);
              // trans->getProfile()->init(path_length,para.v_max, para.a, tg, t12, t13);
            } else {
              tg = tg_trans;
              t12 = trans->getProfile()->getT12();
              t13 = trans->getProfile()->getT13();

              real_t new_omega_max = (get_angle * sgn_angle - 0.5 * rot->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t13) + 0.5 * t12);
              real_t new_alpha = (new_omega_max - rot->getProfile()->getVb()) / t12;

              rot->setNewSpeedAndAccel(0.0, 0.0, new_omega_max, 0.0, new_alpha);
              // rot->getProfile()->init(get_angle*sgn_angle, new_omega_max, new_alpha);
            }
          } else // tg_trans == 0 or tg_rot == 0 or both
          {
            tg = tg_trans + tg_rot;
          }
        }

        RotAndTrans(imsl::PoseV<real_t> Start, imsl::PoseV<real_t> End, PathParameters<real_t> p)
          : RotAndTrans(Start, End, p.v_max, p.a, p.omega_max, p.alpha, p.localizationmode, p.waitTime)
        {
        }

        ~RotAndTrans()
        {
          delete (rot);
          delete (trans);
        }

        imsl::PoseV<real_t> Interpolation(real_t t) override
        {
          // real_t omega, phi;
          if (t < 0.0) {
            Pact = Pstart;
          } else if (t > tg) {
            Pact = Pend;
          } else {
            P_temp = rot->Interpolation(t);
            Pact = trans->Interpolation(t);
            Pact.theta = P_temp.theta;
            Pact.omega = P_temp.omega;
          }
          return Pact;
        }

        void setNewSpeed(real_t t, real_t v, real_t omega) override
        {
          real_t tg_rot,
            tg_trans,
            t12,
            t13;
          if (t < 0.0)
            throw IMSLex(1, "RotAndTrans::setNewSpeed(): t <0");
          // recalculate only the actual segment and segments in the future
          // tg (t_rot1,2, t_trans) of segments in the past is never infinity
          rot->setNewSpeedAndAccel(t, 0.0, omega, 0.0, para.alpha);
          // rot->setNewSpeed(t,v,omega);
          tg_rot = rot->getTg();
          trans->setNewSpeedAndAccel(t, v, 0.0, para.a, 0.0);
          // trans->setNewSpeed(t,v,omega);
          tg_trans = trans->getTg();
          if (tg_trans < tg_rot) {

            tg = tg_rot;
            t12 = rot->getProfile()->getT12();
            t13 = rot->getProfile()->getT13();

            real_t new_s = trans->getProfile()->getSg() - trans->getProfile()->getSb();
            real_t new_v_max = (new_s - 0.5 * trans->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t - t13) + 0.5 * t12);
            real_t new_a = (new_v_max - trans->getProfile()->getVb()) / t12;

            trans->setNewSpeedAndAccel(t, new_v_max, 0.0, std::abs(new_a), 0.0);

            // trans->getProfile()->init(path_length, new_v_max, new_a);
            // trans->getProfile()->init(path_length,para.v_max, para.a, tg, t12, t13);
          } else {

            tg = tg_trans;
            t12 = trans->getProfile()->getT12();
            t13 = trans->getProfile()->getT13();

            real_t new_s = rot->getProfile()->getSg() - rot->getProfile()->getSb();
            real_t new_omega_max = (new_s - 0.5 * rot->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t - t13) + 0.5 * t12);
            real_t new_alpha = (new_omega_max - rot->getProfile()->getVb()) / t12;

            // rot->getProfile()->init(get_angle*sgn_angle, new_omega_max, std::abs(new_alpha));
            rot->setNewSpeedAndAccel(t, 0.0, new_omega_max, 0.0, std::abs(new_alpha));
          }
          // printf("Final tg: %f \n",tg);
          return;
        }

        void setNewSpeedAndAccel(real_t t, real_t v, real_t omega, real_t a, real_t alpha) override
        {
          real_t tg_rot,
            tg_trans,
            t12,
            t13;
          if (t < 0.0)
            throw IMSLex(1, "RotAndTrans::setNewSpeed(): t <0");
          // recalculate only the actual segment and segments in the future
          // tg (t_rot1,2, t_trans) of segments in the past is never infinity
          rot->setNewSpeedAndAccel(t, 0.0, omega, 0.0, alpha);
          // rot->setNewSpeed(t,v,omega);
          tg_rot = rot->getTg();
          trans->setNewSpeedAndAccel(t, v, 0.0, a, 0.0);
          // trans->setNewSpeed(t,v,omega);
          tg_trans = trans->getTg();

          if (tg_trans < tg_rot) {

            tg = tg_rot;
            t12 = rot->getProfile()->getT12();
            t13 = rot->getProfile()->getT13();

            real_t new_s = trans->getProfile()->getSg() - trans->getProfile()->getSb();
            real_t new_v_max = (new_s - 0.5 * trans->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t - t13) + 0.5 * t12);
            real_t new_a = (new_v_max - trans->getProfile()->getVb()) / t12;

            trans->setNewSpeedAndAccel(t, new_v_max, 0.0, std::abs(new_a), 0.0);

            // trans->getProfile()->init(path_length, new_v_max, new_a);
            // trans->getProfile()->init(path_length,para.v_max, para.a, tg, t12, t13);
          } else {

            tg = tg_trans;
            t12 = trans->getProfile()->getT12();
            t13 = trans->getProfile()->getT13();

            real_t new_s = rot->getProfile()->getSg() - rot->getProfile()->getSb();
            real_t new_omega_max = (new_s - 0.5 * rot->getProfile()->getVb() * t12) / ((t13 - t12) + 0.5 * (tg - t - t13) + 0.5 * t12);
            real_t new_alpha = (new_omega_max - rot->getProfile()->getVb()) / t12;

            // rot->getProfile()->init(get_angle*sgn_angle, new_omega_max, std::abs(new_alpha));
            rot->setNewSpeedAndAccel(t, 0.0, new_omega_max, 0.0, std::abs(new_alpha));
          }
          // printf("Final tg: %f \n",tg);

          return;
        }
    }; // end of class RotAndTrans
  } // namespace interpolator
} // namespace imsl

#endif // #define INCLUDED_INTERPOLATOR_H
