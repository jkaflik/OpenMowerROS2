// This code originally comes from
// https://github.com/ClemensElflein/xbot_positioning/blob/09f4e783fa0a445c260f03574120fec1a21f1f0e/src/xbot_positioning_core.h

#pragma once

#include "system_model.hpp"
#include "orientation_measurement_model.hpp"
#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>


namespace OpenMower {
    namespace GPSOrientation {
        typedef double T;

        typedef OpenMower::GPSOrientation::State<T> StateT;
        typedef OpenMower::GPSOrientation::OrientationMeasurement2<T> OrientationMeasurementT2;

        class Core {


        public:
            Core();

            const StateT &predict(double vx, double vr, double dt);

            const StateT &updateOrientation(double vx, double vy, double covariance);

            const StateT &getState();

            void setState(double px, double py, double theta, double vx, double vr);

            const Kalman::Covariance <StateT> &getCovariance();

            void setAntennaOffset(double offset_x, double offset_y);

        public:
            Kalman::ExtendedKalmanFilter <StateT> ekf{};
            SystemModelT sys{};
            PositionModelT pm{};
            OrientationModelT om{};
            OrientationModelT2 om2{};
            SpeedModelT sm{};

            ControlT u{};
            OrientationMeasurementT2 orient_m2{};
            SpeedMeasurementT speed_m{};
        };
    }
}

#endif //SRC_Core_H
