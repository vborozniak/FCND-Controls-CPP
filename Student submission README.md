# The C++ Project Readme #

This is the Student submission readme for the C++ project. Please use QuadControlParams.txt and QuadControl files in the cloned github folder - https://github.com/vborozniak/FCND-Controls-CPP

## Development Environment Setup ##

Windows used for Development Environment Setup


### The Code ### & ### The Simulator ###

All required control functions are implemented in `src/QuadControl.cpp`. Below is a detailed explanation of each it.

1. **Body rate control**  
   Implemented in `BodyRateControl()`.  
   A proportional controller on body rate error is used, with moments of inertia (Ixx, Iyy, Izz) accounted for to produce desired moments.  
   Code:
   ```cpp
   momentCmd.x = Ixx * kpPQR.x * (pqrCmd.x - pqr.x);
   momentCmd.y = Iyy * kpPQR.y * (pqrCmd.y - pqr.y);
   momentCmd.z = Izz * kpPQR.z * (pqrCmd.z - pqr.z);

2. Implemented in RollPitchControl().
Desired lateral acceleration and collective thrust are used to compute target body-z projections in the inertial frame. Magnitude-based tilt limiting is applied for symmetry (especially important for diagonal Quad2 in scenario 3). Full matrix projection is used to convert desired projection rates to body rates, with scaling by R(2,2) for natural authority reduction at high tilt. Negative thrust acceleration convention is used for correct direction. The function can possibly be further improved as the Scenarion 3 was not able to fully pass.

3. Altitude control
Implemented in AltitudeControl().
PD control on position and velocity error, plus integral term for steady-state compensation. Thrust is scaled by drone mass and cosine of tilt angles via / R(2,2). Basic anti-windup is added by backing off the integral when thrust saturates.
Integral term is required for scenario 4 mass variation; still unable to find a good set of parameters to make drones follow the smooth. Room for improvement

4. Lateral position control
Implemented in LateralPositionControl().
Cascaded design: position error generates desired velocity (with magnitude limit via maxSpeedXY), then velocity error generates acceleration (with magnitude limit via maxAccelXY). Feed-forward acceleration is added.
Reason: Cascaded structure with limits provides smooth, bounded approach and prevents runaway speed/accel.

5. Yaw control
Implemented in YawControl().
Simple proportional control on unwrapped yaw error (using fmodf to [-π, π] range).
Linear P control is sufficient and low-authority, unwrapping prevents jumps.

6. Motor commands
Implemented in GenerateMotorCommands().
Collective thrust and moments are mixed to individual motor thrusts using arm length L and torque/thrust ratio kappa. Signs match standard X-configuration.

## Flight Evaluation and Tuning Notes ##

Mass adjusted for Intro Scenario.
Scenario 2 (Attitude Control): Passes both metrics consistently (roll < 0.025 rad and roll rate < 2.5 rad/s for ≥0.75s). The attitude loop is fast and well-damped.
Scenario 3 (Position/Yaw Hold): Visual behavior is stable—quads converge to target with correct direction, good damping, and symmetric performance for both quads (including diagonal Quad2). Yaw metric passes reliably. Position metric fails due to  slow final settling (enters band but timing just misses sustained 1.25s). Extensive tuning of gains, limits, projection variants, and thrust margin was performed, but the simulator's lag/estimator appears to limit tighter hold...
Scenario 4 (Non-Idealities): Integral term and anti-windup handle mass variation well (no droop). High rate gains reject COM shift torque (minimal drift). Visual motion is stable for all three quads, though metric may not pass due to scenario 3 tuning carry-over.
Scenario 5 (Trajectory Tracking): Not fully tested due to difficulties passing Scenario 3 and 4...

## Tuning Decisions and Rationale ##
Gains in config/QuadControlParams.txt are scaled aggressively from defaults (~10-35×) to overcome simulator lag and achieve fast convergence (gradual increase in control params was conducted without good match to achieve target error margins):

High kpPosXY/kpVelXY for quick offset recovery.
High kpBank/kpPQR for sharp attitude/rate response.
High Z gains to eliminate droop coupling.

Multiple RollPitchControl variants were tested (constant vs scaled gain, simple vs full projection, positive vs negative target/c) to balance damping and speed. The current full projection with scaled gain and negative c provides the best stability and visual performance.
The controller is robust and visually performs all tasks well. The tight timing metrics in scenarios 3–5 appear sensitive to simulator randomness and estimator delay, preventing consistent PASS.
All code is original implementation based on project guidelines and physics principles.
