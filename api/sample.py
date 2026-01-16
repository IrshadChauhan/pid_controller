import time
import math

try:
    import sim as vrep
except:
    print('Check if sim.py and library files are in this folder!')

class PIDController:
    def _init_(self, kp, ki, kd, setpoint):
        # Task 1: Initialize components and error storage
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
       
        # Task 1: Store previous error values for Integral & Derivative
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0: dt = 0.01
       
        error = self.setpoint - current_value
       
        # Task 1: Proportional term
        P = self.kp * error
       
        # Task 1: Integral term (Accumulated error)
        self.integral += error * dt
        I = self.ki * self.integral
       
        # Task 1: Derivative term (Change in error)
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative
       
        self.prev_error = error
        self.last_time = now
        output = P + I + D
       
        # Task 1: Output Thresholding (Safety Limit of 0.3 m/s)
        if output > 0.3: output = 0.3
        if output < -0.3: output = -0.3
       
        return output

# --- Connection Setup ---
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print('Connected to CoppeliaSim - Final Submission Setup')

    # Handles
    _, left_motor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
    _, right_motor = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
    _, side_sonar = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor3', vrep.simx_opmode_blocking)

    # Task 2: Tuning - Safe Calculated Parameters
    # base_speed = 0.2: Slow enough to prevent momentum crashes in corners.
    # kp = 2.5: Sharp proportional reaction.
    # kd = 1.0: High derivative to anticipate corners and stabilize.
    pid = PIDController(kp=2.5, ki=0.01, kd=1.0, setpoint=0.5)
    base_speed = 2.0

    try:
        while True:
            res, detected, point, _, _ = vrep.simxReadProximitySensor(clientID, side_sonar, vrep.simx_opmode_streaming)
           
            if not detected:
                # Task 1: Wandering behavior
                v_left = v_right = base_speed
            else:
                # Task 1: PID Wall Following
                distance = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                correction = pid.update(distance)
               
                # Live logging for Task 2 "Testing Evidence"
                print(f"Distance: {distance:.3f} | Correction: {correction:.3f}")

                # Steering Logic: Turns LEFT away from wall on the right
                v_left = base_speed - correction
                v_right = base_speed + correction

            vrep.simxSetJointTargetVelocity(clientID, left_motor, v_left, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(clientID, right_motor, v_right, vrep.simx_opmode_oneshot)
            time.sleep(0.05)

    except KeyboardInterrupt:
        # Safety stop on exit
        vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetVelocity(clientID, right_motor, 0, vrep.simx_opmode_blocking)
        vrep.simxFinish(clientID)
else:
    print('Failed connecting. Ensure CoppeliaSim Play button is pressed!')