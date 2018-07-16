Writeup from Ruberic 

1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. 
Included herein. 

2. Implemented body rate control in C++.  The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.
```
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
```

3. Implement roll pitch control in C++.  The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a 
body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should 
be accounted for when calculating the target angles.
```
   // python reference
    // 	R = euler2RM(attitude[0], attitude[1], attitude[2])
    //    c_d = thrust_cmd/DRONE_MASS_KG
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
```
4. Implement altitude controller in C++.  The controller should use both the down position and the down velocity to command thrust. Ensure that the output 
value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.
Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

```
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
``` 

5. Implement lateral position control in C++.  The controller should use the local NE position and velocity to generate a commanded local acceleration.

``` 
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
```

6. Implement yaw control in C++.  The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).
```
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
    //return rad/s
    yawRateCmd = kpYaw * yaw_error_rate;
```

7. Implement calculating the motor commands given commanded thrust and moments in C++. The thrust and moments should be converted to the appropriate 4 different 
desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.
```
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
```

8. Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.  
Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able 
to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used). 

Mission completed! 
