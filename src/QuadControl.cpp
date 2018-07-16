//
//  QuadControl.cpp
//  Controller.py reference provided by Udacity https://github.com/udacity/FCND-Controls/blob/solution/controller.py
//
#include "QuadControl.hpp"
#include <iostream>

#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include "Math/Angles.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

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
    //   desCollectiveThrust: desired collective thrust [N]
    //   desMoment: desired rotation moment about each axis [N m]
    // OUTPUT:
    //   set class member variable cmd (class variable for graphing) where
    //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
    
    // HINTS:
    // - you can access parts of desMoment via e.g. desMoment.x
    // You'll need the arm length parameter L, and the drag/thrust ratio kappa
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // python reference
    // ind_min = np.argmin(np.abs(np.array(time_trajectory)-current_time))
    //    time_ref = time_trajectory[ind_min]
    //    if current_time < time_ref:
    //        position0 = position_trajectory[ind_min-1]
    //        position1 = position_trajectory[ind_min]
    //        time0 = time_trajectory[ind_min-1]
    //        time1 = time_trajectory[ind_min]
    //        yaw_cmd = yaw_trajectory[ind_min-1]
    //    else:
    //        yaw_cmd = yaw_trajectory[ind_min]
    //        if ind_min >= len(position_trajectory)-1:
    //            position0 = position_trajectory[ind_min]
    //            position1 = position_trajectory[ind_min]
    //            time0 = 0.0
    //            time1 = 1.0
    //        else:
    //            position0 = position_trajectory[ind_min]
    //            position1 = position_trajectory[ind_min+1]
    //            time0 = time_trajectory[ind_min]
    //            time1 = time_trajectory[ind_min+1]
    //    position_cmd = (position1-position0)* \
    //                    (current_time-time0)/(time1-time0)+position0
    //    velocity_cmd = (position1-position0)/(time1-time0)
    //    return (position_cmd,velocity_cmd,yaw_cmd)
    //
    //collective thrust
    float ctc = collThrustCmd;
    //rotation moment of x axis
    float x_a = (momentCmd.x) / (L / sqrtf(2.0f));
    //rotation moment of y axis
    float y_a = (momentCmd.y) / (L / sqrtf(2.0f));
    //rotation moment of z axis
    float z_a = (momentCmd.z) / -kappa;
    // front left thrust
    cmd.desiredThrustsN[0] = (ctc + x_a + y_a + z_a) / 4.f;
    // front right thrust
    cmd.desiredThrustsN[1] = (ctc - x_a + y_a - z_a) / 4.f;
    // rear right thrust
    cmd.desiredThrustsN[3] = (ctc - x_a - y_a + z_a) / 4.f;
    // rear left thrust
    cmd.desiredThrustsN[2] = (ctc + x_a - y_a - z_a) / 4.f;
    
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
    
    V3F momentCmd;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    //  python reference
    //  Kp_rate = np.array([self.Kp_p, self.Kp_q, self.Kp_r])
    //  rate_error = body_rate_cmd - body_rate
    //  moment_cmd = MOI * np.multiply(Kp_rate, rate_error)
    //  if np.linalg.norm(moment_cmd) > MAX_TORQUE:
    //     moment_cmd = moment_cmd*MAX_TORQUE/np.linalg.norm(moment_cmd)
    //  return moment_cmd
    
    // include gain * (desired body rates less currert or estimated body rate)
    V3F body_rates = kpPQR * (pqrCmd - pqr);
    //moments of inertia * body rates for x
    momentCmd[0] = Ixx * body_rates.x;
    //moments of inertia * body rates for y
    momentCmd[1] = Iyy * body_rates.y;
    //moments of inertia * body rates for z
    momentCmd[2] = Izz * body_rates.z;
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
    
    V3F pqrCmd;
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // python reference
    //        R = euler2RM(attitude[0], attitude[1], attitude[2])
    //     c_d = thrust_cmd/DRONE_MASS_KG
    //     if thrust_cmd > 0.0:
    //        target_R13 = -np.clip(acceleration_cmd[0].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[0].item()/c_d, -self.max_tilt), self.max_tilt)
    //        target_R23 = -np.clip(acceleration_cmd[1].item()/c_d, -self.max_tilt, self.max_tilt) #-min(max(acceleration_cmd[1].item()/c_d, -self.max_tilt), self.max_tilt)
    //
    //        p_cmd = (1/R[2, 2]) * \
    //                (-R[1, 0] * self.Kp_roll * (R[0, 2]-target_R13) + \
    //                 R[0, 0] * self.Kp_pitch * (R[1, 2]-target_R23))
    //        q_cmd = (1/R[2, 2]) * \
    //                (-R[1, 1] * self.Kp_roll * (R[0, 2]-target_R13) + \
    //                 R[0, 1] * self.Kp_pitch * (R[1, 2]-target_R23))
    //    else:  # Otherwise command no rate
    //        print("negative thrust command")
    //        p_cmd = 0.0
    //        q_cmd = 0.0
    //        thrust_cmd = 0.0
    //    return np.array([p_cmd, q_cmd])
    
    // convert from Newtons to acceleration
    float col = collThrustCmd;
    // constrain pitch
    float target_R13 = -CONSTRAIN(accelCmd.x / col, -maxTiltAngle, maxTiltAngle);
    float target_R23 = -CONSTRAIN(accelCmd.y / col, -maxTiltAngle, maxTiltAngle);
    
    // if command no rate, negative thrust command
    if (collThrustCmd < 0) {
        target_R13 = 0;
        target_R23 = 0;
    }
    
    //set target
    float target_R13_Cmd = kpBank * (target_R13 - R(0, 2));
    float target_R23_Cmd = kpBank * (target_R23 - R(1, 2));
    
    // issue a thrust command for x and y axis, if negative thrust then computes as zero.
    pqrCmd.x = (R(1, 0) * target_R13_Cmd - R(0, 0) * target_R23_Cmd) / R(2, 2);
    pqrCmd.y = (R(1, 1) * target_R13_Cmd - R(0, 1) * target_R23_Cmd) / R(2, 2);
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd,
                                   float velZCmd,
                                   float posZ,
                                   float velZ,
                                   Quaternion<float> attitude,
                                   float accelZCmd,
                                   float dt) {
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
    
    Mat3x3F R = attitude.RotationMatrix_IwrtB();
    float thrust = 0;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // python reference
    // hdot_cmd = self.Kp_alt * (altitude_cmd - altitude) + vertical_velocity_cmd
    // hdot_cmd = np.clip(hdot_cmd, -self.max_descent_rate, self.max_ascent_rate)
    // acceleration_cmd = acceleration_ff + self.Kp_hdot*(hdot_cmd - vertical_velocity)
    // R33 = np.cos(attitude[0]) * np.cos(attitude[1])
    // thrust = DRONE_MASS_KG * acceleration_cmd / R33
    // if thrust > MAX_THRUST:
    // thrust = MAX_THRUST
    // elif thrust < 0.0:
    // thrust = 0.0
    // return thrust
    
    float posZError = posZCmd - posZ;
    // desired unconstrained velocity in NED [m]
    velZCmd += kpPosZ * posZError;
    // constrain the veolicty between ascent and descent rates
    velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);
    // computer the error rate based on the position and time
    integratedAltitudeError += posZError * dt;
    //current velocity in NED [m] - desired volocity in NED [m]
    float velZdiff = velZCmd - velZ;
    float Grav = CONST_GRAVITY;
    // compute the acceleration
    float zRate = (accelZCmd + kpVelZ * velZdiff + KiPosZ * integratedAltitudeError - Grav);
    //return a force not an accelaration
    thrust = -mass * zRate / R(2, 2);
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
    V3F accelCmd = accelCmdFF;
    
    ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    //  Reference to FCND-Controls Python example code: https://github.com/udacity/FCND-Controls/blob/solution/controller.py
    //
    //  velocity_cmd = self.Kp_pos*(local_position_cmd-local_position)
    //
    // Limit speed
    //    velocity_norm = np.sqrt(velocity_cmd[0]*velocity_cmd[0] + \
    //                            velocity_cmd[1]*velocity_cmd[1])
    //
    //    if velocity_norm > self.max_speed:
    //        velocity_cmd = velocity_cmd*self.max_speed/velocity_norm
    //
    //    acceleration_cmd = acceleration_ff + \
    //                        self.Kp_pos*(local_position_cmd-local_position) +\
    //                        self.Kp_vel*(local_velocity_cmd-local_velocity)
    //
    //    return acceleration_cmd
    
    velCmd = velCmd +(kpPosXY * (posCmd - pos));
    // desired velocity
    float velocity_norm = sqrt(velCmd[0] * velCmd[0] + velCmd[1] * velCmd[1]);
    //limit the maximum velocity to maxSpeedXY
    if (velCmd.mag() > maxSpeedXY){
        velCmd = ((velCmd.norm() * maxSpeedXY)/velocity_norm);
    }
    accelCmd = accelCmdFF + ( kpPosXY * (posCmd - pos)) + (kpVelXY * (velCmd - vel));
    // limit the maximum acceleration to maxAccelXY
    if (accelCmd.mag() > maxAccelXY) {
        accelCmd = accelCmd.norm() * maxAccelXY;
    }
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
    // reference python
    // yaw_cmd = np.mod(yaw_cmd, 2.0*np.pi)
    // yaw_error = yaw_cmd - yaw
    // if yaw_error > np.pi:
    //    yaw_error = yaw_error - 2.0*np.pi
    //    elif yaw_error < -np.pi:
    //    yaw_error = yaw_error + 2.0*np.pi
    //    yawrate_cmd = self.Kp_yaw*yaw_error
    //    return yawrate_cmd
    //
    float yaw_error_rate = yawCmd - yaw;
    // normalize angle to -pi to pi
    // call to inline float AngleNormF(float angle)
    yaw_error_rate = AngleNormF(yaw_error_rate);
    //return desired yaw rate in rad/s
    yawRateCmd = kpYaw * yaw_error_rate;
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
    return yawRateCmd;
    
}

VehicleCommand QuadControl::RunControl(float dt, float simTime) {
    curTrajPoint = GetNextTrajectoryPoint(simTime);
    
    float collThrustCmd = AltitudeControl(curTrajPoint.position.z,
                                          curTrajPoint.velocity.z,
                                          estPos.z,
                                          estVel.z,
                                          estAtt,
                                          curTrajPoint.accel.z,
                                          dt);
    
    // reserve some thrust margin for angle control
    float thrustMargin = .1f * (maxMotorThrust - minMotorThrust);
    collThrustCmd =
    CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin) * 4.f, (maxMotorThrust - thrustMargin) * 4.f);
    
    V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
    
    V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
    desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());
    
    V3F desMoment = BodyRateControl(desOmega, estOmega);
    
    return GenerateMotorCommands(collThrustCmd, desMoment);
}
