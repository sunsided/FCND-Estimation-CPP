#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

#define FUN_AND_PROFIT

#ifdef FUN_AND_PROFIT

V3F LimitVectorMagnitude(const V3F &command, const float maximumMagnitude) {
    // Normalizes a vector command by a given maximum magnitude.
    const auto magnitude = command.mag();
    if (magnitude <= maximumMagnitude || maximumMagnitude <= 0.0F) {
        return command;
    }
    return command * maximumMagnitude / magnitude;
}

#else

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedLocalVariable"

V3F LimitVectorMagnitude(const V3F &command, const float maximumMagnitude) {
    // Everything is as normal as it gets.
    return command;
}

#pragma clang diagnostic pop

#endif

float WrapAngleRadians(float angle) {
    // Ensures an angle is within -pi .. pi range.

    static const auto pi = static_cast<float>(M_PI);
    static const auto pi2 = 2.0F * static_cast<float>(M_PI);

    while (angle > pi) {
        angle -= pi2;
    }
    while (angle < -pi) {
        angle += pi2;
    }

    return angle;
}

void QuadControl::Init() {
    BaseController::Init();

    // variables needed for integral control
    integratedAltitudeError = 0;

#ifndef __PX4_NUTTX
    // Load params from simulator parameter system
    ParamsHandle config = SimpleConfig::GetInstance();

    // Load parameters (default to 0)
    kpPosXY = config->Get(_config + ".kpPosXY", 0);
    kpPosZ = config->Get(_config + ".kpPosZ", 0);
    KiPosZ = config->Get(_config + ".KiPosZ", 0);

    kpVelXY = config->Get(_config + ".kpVelXY", 0);
    kpVelZ = config->Get(_config + ".kpVelZ", 0);

    kpBank = config->Get(_config + ".kpBank", 0);
    kpYaw = config->Get(_config + ".kpYaw", 0);

    kpPQR = config->Get(_config + ".kpPQR", V3F());

    maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
    maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
    maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
    maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

    maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

    minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
    maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);

    maxTorque = config->Get(_config + ".maxTorque", 1);
#else
    // load params from PX4 parameter system
    //TODO
    param_get(param_find("MC_PITCH_P"), &Kp_bank);
    param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd) {
    // Convert a desired 3-axis moment and collective thrust command to
    //   individual motor thrust commands
    // INPUTS:
    //   collThrustCmd: desired collective thrust [N]
    //   momentCmd: desired rotation moment about each axis [N m]
    // OUTPUT:
    //   set class member variable cmd (class variable for graphing) where
    //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

    // HINTS:
    // - you can access parts of momentCmd via e.g. momentCmd.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // Perpendicular distance (d_perp, l):
    // The rotors are assembled L units away from the center of mass,
    // at a 45° angle to the body frame coordinate system.
    // Thus, the perpendicular distance l = L * cos(45°).
    // Since cos(45°) = 1/√2, we have
    const auto d_perp = L * static_cast<float>(M_SQRT1_2);

    // The goal of this method is to take a total thrust F and
    // distribute it across all rotors, such that the specified rotational
    // moment is obtained.
    // Note that collThrustCmd = F_1 + F_1 + F_2 + F_3

    // The core problem with this task is that the code doesn't give
    // the thrust and torque coefficients described in the lectures, but herein lies the
    // core realization required to solve this problem:
    //
    // We know that
    //
    //    F   = kf * omega^2
    //    tau = km * omega^2
    //
    // Thus thrust and torque are coupled by the angular velocity.
    // Equating both gives
    //
    //    F / kf  = tau / km
    //    tau / F = km / kf
    //
    // From the source code (BaseController.h) and the physical parameters (QuadPhysicalParams.txt)
    // we know that kappa is "torque (Nm) produced by motor per N of thrust produced", thus
    //
    //    kappa = tau / F
    //
    // and specifically, "torque = kappa * thrust". This also follows directly from the equations above:
    //
    //    tau = kappa * F
    //
    // The rest of the problem is pretty straightforward, we have:
    //
    //    F =  F1 + F2 + F3 + F3
    //
    // and for the roll and pitch moments:
    //
    //    tau_x = (F1 - F2 - F3 + F4) * l
    //    tau_y = (F1 + F2 - F3 - F4) * l
    //
    // The yaw moment is a result of the (signed!) moments induced by each motor
    // (do note that the exercise notebook to the lesson ignores the signs, which leads to extra confusion):
    //
    //    tau_z = tau_1 - tau_2 + tau_3 - tau_4
    //
    // Applying the kappa trick mentioned above, we find that
    //
    //   tau_z = (F1 - F2 + F3 - F4) * kappa
    //
    // At this point, we have four unknowns F1, F2, F3, F4, as well as four
    // knowns F, tau_x, tau_y, tau_z. If we build a linear system and solve for F1 .. F4,
    // we find that
    //
    //    F1 = 1/4 * (F + tau_x/l + tau_y/l + tau_z/kappa)
    //    F2 = 1/4 * (F - tau_x/l + tau_y/l - tau_z/kappa)
    //    F3 = 1/4 * (F - tau_x/l - tau_y/l + tau_z/kappa)
    //    F4 = 1/4 * (F + tau_x/l - tau_y/l - tau_z/kappa)
    //
    // A final twist is that the rotor numbering from the notebooks differ
    // from the one in the simulator; specifically, rotor 3 and 4 are swapped.
    // In the code below, the assignments are swapped accordingly.

    const auto Fc =  collThrustCmd;             // force from collective thrust
    const auto Fp =  momentCmd.x / d_perp;      // force from pitch moment
    const auto Fq =  momentCmd.y / d_perp;      // force from roll moment
    const auto Fr = -momentCmd.z / kappa;       // force from yaw moment

    const auto F1 = 0.25 * (Fc + Fp + Fq + Fr);
    const auto F2 = 0.25 * (Fc - Fp + Fq - Fr);
    const auto F3 = 0.25 * (Fc + Fp - Fq - Fr);
    const auto F4 = 0.25 * (Fc - Fp - Fq + Fr);

    // It appears that we don't have to concern ourselves with the limiting the thrust range here.
    // Concretely, if we do, this will make the simulator goals harder to pass and requires extremely
    // high controller gains.

#ifndef LIMIT_THRUST_RANGE

    cmd.desiredThrustsN[0] = F1; // front left
    cmd.desiredThrustsN[1] = F2; // front right
    cmd.desiredThrustsN[2] = F3; // rear left
    cmd.desiredThrustsN[3] = F4; // rear right

#else

    cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust); // front left
    cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust); // front right
    cmd.desiredThrustsN[2] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust); // rear left
    cmd.desiredThrustsN[3] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust); // rear right

#endif

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr) {
    // Calculate a desired 3-axis moment given a desired and current body rate
    // INPUTS:
    //   pqrCmd: desired body rates [rad/s]
    //   pqr: current or estimated body rates [rad/s]
    // OUTPUT:
    //   return a V3F containing the desired moments for each of the 3 axes

    // HINTS:
    //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
    //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
    //  - you'll also need the gain parameter kpPQR (it's a V3F)

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    const auto rateError = pqrCmd - pqr;

    const V3F moi { Ixx, Iyy, Izz };
    auto momentCmd = moi * (kpPQR * rateError);

    momentCmd = LimitVectorMagnitude(momentCmd, maxTorque);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return momentCmd;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd) {
    // Calculate a desired pitch and roll angle rates based on a desired global
    //   lateral acceleration, the current attitude of the quad, and desired
    //   collective thrust command
    // INPUTS:
    //   accelCmd: desired acceleration in global XY coordinates [m/s2]
    //   attitude: current or estimated attitude of the vehicle
    //   collThrustCmd: desired collective thrust of the quad [N]
    // OUTPUT:
    //   return a V3F containing the desired pitch and roll rates. The Z
    //     element of the V3F should be left at its default value (0)

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the roll/pitch gain kpBank
    //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

    V3F pqrCmd { 0.0, 0.0, 0.0 };
    Mat3x3F R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    if (collThrustCmd <= 0) {
        return pqrCmd;
    }

    // In NED, down is the new up.
    const auto acceleration = -collThrustCmd / mass;

    // Target roll and pitch rates.
    const auto b_x_c_target = CONSTRAIN(accelCmd.x / acceleration, -maxTiltAngle, maxTiltAngle);
    const auto b_y_c_target = CONSTRAIN(accelCmd.y / acceleration, -maxTiltAngle, maxTiltAngle);

    // Actual roll and pitch values.
    const auto b_x = R(0,2);
    const auto b_y = R(1,2);

    // Error terms
    const auto b_x_c_error = b_x_c_target - b_x;
    const auto b_y_c_error = b_y_c_target - b_y;

    // P controller for roll and pitch rates.
    const auto b_x_commanded_dot = kpBank * b_x_c_error;
    const auto b_y_commanded_dot = kpBank * b_y_c_error;

    // Convert to rates in the body frame.
    pqrCmd.x = (R(1,0) * b_x_commanded_dot - R(0,0) * b_y_commanded_dot) / R(2,2);
    pqrCmd.y = (R(1,1) * b_x_commanded_dot - R(0,1) * b_y_commanded_dot) / R(2,2);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude,
                                   float accelZCmd, float dt) {
    // Calculate desired quad thrust based on altitude setpoint, actual altitude,
    //   vertical velocity setpoint, actual vertical velocity, and a vertical
    //   acceleration feed-forward command
    // INPUTS:
    //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
    //   posZ, velZ: current vertical position and velocity in NED [m]
    //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
    //   dt: the time step of the measurements [seconds]
    // OUTPUT:
    //   return a collective thrust command in [N]

    // HINTS:
    //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
    //  - you'll need the gain parameters kpPosZ and kpVelZ
    //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
    //  - make sure to return a force, not an acceleration
    //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

    auto R = attitude.RotationMatrix_IwrtB();

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    const auto gravity = static_cast<float>(CONST_GRAVITY);

    // Position and velocity errors
    const auto z_err     = posZCmd - posZ;      // z position error (for P controller)
    const auto z_dot_err = velZCmd - velZ;      // z velocity error (for D controller)

    // Integrate the error for the I controller.
    integratedAltitudeError += z_err * dt;

    // PID controller (with feedforward)
    const auto p_term = kpPosZ * z_err;
    const auto d_term = kpVelZ * z_dot_err;
    const auto i_term = KiPosZ * integratedAltitudeError;
    const auto z_dot_dot_c = p_term + i_term + d_term + accelZCmd;

    // Convert from world frame to body frame.
    const auto b_z = R(2, 2);
    const auto c = (z_dot_dot_c - gravity) / b_z;

    // Limiting for fun and profit.
    const auto ascLimit = maxAscentRate / dt;
    const auto descLimit = maxDescentRate / dt;
    const auto thrust = -mass * CONSTRAIN(c, -descLimit, ascLimit);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF) {
    // Calculate a desired horizontal acceleration based on
    //  desired lateral position/velocity/acceleration and current pose
    // INPUTS:
    //   posCmd: desired position, in NED [m]
    //   velCmd: desired velocity, in NED [m/s]
    //   pos: current position, NED [m]
    //   vel: current velocity, NED [m/s]
    //   accelCmdFF: feed-forward acceleration, NED [m/s2]
    // OUTPUT:
    //   return a V3F with desired horizontal accelerations.
    //     the Z component should be 0
    // HINTS:
    //  - use the gain parameters kpPosXY and kpVelXY
    //  - make sure you limit the maximum horizontal velocity and acceleration
    //    to maxSpeedXY and maxAccelXY

    // make sure we don't have any incoming z-component
    accelCmdFF.z = 0;
    velCmd.z = 0;
    posCmd.z = pos.z;

    // we initialize the returned desired acceleration to the feed-forward value.
    // Make sure to _add_, not simply replace, the result of your controller
    // to this variable
    auto accelCmd = accelCmdFF;

    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    // Apply range limiting.
    vel = LimitVectorMagnitude(vel, maxSpeedXY);

    // Note that for lateral position control, we keep our fingers away from
    // the z position - hence a zero gain for it.
    const V3F kpP { kpPosXY, kpPosXY, 0.f };
    const V3F kdV { kpVelXY, kpVelXY, 0.f };

    // Position and velocity errors
    const auto xy_err = posCmd - pos;
    const auto xy_dot_err =  velCmd - vel;

    // PD controller
    const auto p_term = kpP * xy_err;
    const auto d_term = kdV * xy_dot_err;

    // PD controller (with feedforward)
    // Note that the feedforward term is already included in accelCmd defined above.
    accelCmd += p_term + d_term;

    // Again, apply saturation.
    accelCmd = LimitVectorMagnitude(accelCmd, maxAccelXY);

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw) {
    // Calculate a desired yaw rate to control yaw to yawCmd
    // INPUTS:
    //   yawCmd: commanded yaw [rad]
    //   yaw: current yaw [rad]
    // OUTPUT:
    //   return a desired yaw rate [rad/s]
    // HINTS:
    //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b].
    //  - use the yaw control gain parameter kpYaw

    float yawRateCmd = 0;
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    const float yaw_err = WrapAngleRadians(yawCmd - yaw);
    yawRateCmd = kpYaw * yaw_err;

    /////////////////////////////// END STUDENT CODE ////////////////////////////

    return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime) {
    curTrajPoint = GetNextTrajectoryPoint(simTime);

    float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt,
                                          curTrajPoint.accel.z, dt);

    // reserve some thrust margin for angle control
    float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
    collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f,
                              (maxMotorThrust - thrustMargin) * 4.f);

    V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel,
                                        curTrajPoint.accel);

    V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
    desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

    V3F desMoment = BodyRateControl(desOmega, estOmega);

    return GenerateMotorCommands(collThrustCmd, desMoment);
}
