// This code originally comes from
// https://github.com/ClemensElflein/xbot_positioning/blob/09f4e783fa0a445c260f03574120fec1a21f1f0e/src/xbot_positioning_core.cpp

#include "core.hpp"


const OpenMower::GPSOrientation::StateT &OpenMower::GPSOrientation::Core::predict(double vx, double vr, double dt) {
    sys.setDt(dt);
    u.v() = vx;
    u.dtheta() = vr;
    return ekf.predict(sys, u);
}

const OpenMower::GPSOrientation::StateT &
OpenMower::GPSOrientation::Core::updateOrientation(double vx, double vy, double covariance) {
    orient_m2.vx() = vx;
    orient_m2.vy() = vy;

    Kalman::Covariance <OrientationMeasurementT2> c;
    c.setIdentity();
    c *= covariance;

    om2.setCovariance(c);

    return ekf.update(om2, orient_m2);
}

const OpenMower::GPSOrientation::StateT &OpenMower::GPSOrientation::Core::getState() {
    return ekf.getState();
}

const Kalman::Covariance <OpenMower::GPSOrientation::StateT> &OpenMower::GPSOrientation::Core::getCovariance() {
    return ekf.getCovariance();
}

OpenMower::GPSOrientation::Core::Core() {
//    Kalman::Covariance<StateT> c;
//    c.setIdentity();
//    c *= 0.001;
//    sys.setCovariance(c);
    setState(0, 0, 0, 0, 0);
}

void OpenMower::GPSOrientation::Core::setState(double px, double py, double theta, double vx, double vr) {
    StateT x;
    x.setZero();
    x.x() = px;
    x.y() = py;
    x.theta() = theta;
    x.vx() = vx;
    x.vr() = vr;
    this->ekf.init(x);
    Kalman::Covariance <StateT> c;
    c.setIdentity();
    this->ekf.setCovariance(c);
}

void OpenMower::GPSOrientation::Core::setAntennaOffset(double offset_x, double offset_y) {
    pm.antenna_offset_x = om2.antenna_offset_x = offset_x;
    pm.antenna_offset_y = om2.antenna_offset_y = offset_y;
}

