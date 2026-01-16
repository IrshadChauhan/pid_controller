import time
import math

try:
    import sim as vrep
except:
    print('ERROR: sim.py not found')

# ==================================================
# PID CONTROLLER
# ==================================================
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, current_value):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            dt = 0.01

        error = self.setpoint - current_value

        # Proportional
        P = self.kp * error

        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 1.0), -1.0)
        I = self.ki * self.integral

        # Derivative
        D = self.kd * (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = now

        output = P + I + D

        # Turn speed safety limit
        output = max(min(output, 0.3), -0.3)

        return output

# ==================================================
# CONNECTION
# ==================================================
vrep.simxFinish(-1)
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID == -1:
    print('ERROR: Cannot connect to CoppeliaSim')
    exit()

print('Connected to CoppeliaSim')

# Handles
_, left_motor = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
_, right_motor = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
_, side_sonar = vrep.simxGetObjectHandle(
    clientID, 'Pioneer_p3dx_ultrasonicSensor3', vrep.simx_opmode_blocking)

# ==================================================
# PID TUNING (FINAL)
# ==================================================
pid = PIDController(
    kp=1.4,
    ki=0.01,
    kd=1.2,
    setpoint=0.5
)

base_speed = 1.3

SAFE_DIST = 0.45
DANGER_DIST = 0.30

# Initialise streaming
vrep.simxReadProximitySensor(
    clientID, side_sonar, vrep.simx_opmode_streaming)
time.sleep(0.2)

# ==================================================
# MAIN LOOP
# ==================================================
try:
    while True:
        res, detected, point, _, _ = vrep.simxReadProximitySensor(
            clientID, side_sonar, vrep.simx_opmode_buffer)

        if detected:
            distance = math.sqrt(
                point[0]**2 + point[1]**2 + point[2]**2)

            # ---------------- DANGER ZONE ----------------
            if distance < DANGER_DIST:
                # Emergency turn away from wall
                v_left = -0.5
                v_right = 0.8
                print("⚠️ DANGER: ESCAPING WALL")

            # ---------------- NORMAL PID ----------------
            else:
                correction = pid.update(distance)

                # Reduce speed near wall (corner handling)
                speed = base_speed * (distance / SAFE_DIST)
                speed = max(speed, 0.5)

                v_left = speed - correction
                v_right = speed + correction

                print(f"Dist: {distance:.2f} | Corr: {correction:.2f}")

        else:
            # No wall detected → slow forward motion
            v_left = base_speed
            v_right = base_speed

        vrep.simxSetJointTargetVelocity(
            clientID, left_motor, v_left, vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(
            clientID, right_motor, v_right, vrep.simx_opmode_oneshot)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping robot...")
    vrep.simxSetJointTargetVelocity(
        clientID, left_motor, 0, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetVelocity(
        clientID, right_motor, 0, vrep.simx_opmode_blocking)
    vrep.simxFinish(clientID)
