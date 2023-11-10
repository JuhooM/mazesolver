from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# constants
MAX_SPEED = 4 # speed of robot
SENSOR_THRESHOLD = 80 # sensitivity of sensors surrounding robot for obstacle detection
UNIT_SIZE = 0.125 # size of 1 maze node in mmeters
WHEEL_RADIUS = 0.02002 # radius of robots wheel in mmeters
AXLE_LENGTH = 0.0568 # lenght of the robots axle of wheels in mmeters
timeStep = 1 # time step of the current world.

# variables
pos = [0, 0] # change in sensor position
dis = [0, 0] # change in distance travelled
dirvec = [0.0, 0.0, 0.0] # position vector of robot [x axis position(robot face at start)-meter, y axis position-meter, orientation-rad]

# get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# get a handler to the position sensors and enable them.
left_position_sensor = robot.getDevice('left wheel sensor')
right_position_sensor = robot.getDevice('right wheel sensor')
left_position_sensor.enable(timeStep)
right_position_sensor.enable(timeStep)

# ground sensor
gs = robot.getDevice("gs0")
gs.enable(timeStep)

# initialize distance sensors
ds = robot.getDevice("ds")
ds.enable(timeStep)
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timeStep)

# stop robot movement function
def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

# delay time function for delay_ms
def delay(delay_ms):
    initTime = robot.getTime()
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > delay_ms:
            break
            
# return distance travelled of wheels from previous function call to current time, also update dirvec, par turn true if you are turning otherwise false
def updateVector():
        # update position of wheels
        pos[0] = abs(left_position_sensor.getValue())
        pos[1] = abs(right_position_sensor.getValue())
        
        # calculate distance travelled from previous values
        distance1 = pos[0] * WHEEL_RADIUS - dis[0]
        distance2 = pos[1] * WHEEL_RADIUS - dis[1]
        avgdist = (abs(distance1)+abs(distance2))/2

        # update distance travelled in meters
        dis[0] = pos[0] * WHEEL_RADIUS
        dis[1] = pos[1] * WHEEL_RADIUS
        
        # update dirvec
        if distance1 <= 0 and distance2 >= 0:
            dirvec[2] += (abs(distance1) + abs(distance2))/AXLE_LENGTH;
        elif distance1 > 0 and distance2 < 0:
            dirvec[2] -= (abs(distance1) + abs(distance2))/AXLE_LENGTH;
        else:
            dirvec[0] += avgdist * math.cos(dirvec[2])
            dirvec[1] += avgdist * math.sin(dirvec[2])
        
        print(dirvec)
        print(" ")

# turn right turn amount degrees
def turn_right(turn_amount_deg):
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    revolutions = (turn_amount_deg/360) * math.pi * AXLE_LENGTH / WHEEL_RADIUS

    left_wheel_revolutions = left_position_sensor.getValue()
    right_wheel_revolutions = right_position_sensor.getValue()

    left_motor.setVelocity(MAX_SPEED * 0.33)
    right_motor.setVelocity(-MAX_SPEED * 0.33)

    left_wheel_revolutions += revolutions
    right_wheel_revolutions -= revolutions
    
    left_motor.setPosition(left_wheel_revolutions)
    right_motor.setPosition(right_wheel_revolutions)
    
    wait_turn()

# turn left turn amount degrees
def turn_left(turn_amount_deg):
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    revolutions = (turn_amount_deg/360) * math.pi * AXLE_LENGTH / WHEEL_RADIUS

    left_wheel_revolutions = left_position_sensor.getValue()
    right_wheel_revolutions = right_position_sensor.getValue()

    left_motor.setVelocity(-MAX_SPEED * 0.33)
    right_motor.setVelocity(MAX_SPEED * 0.33)

    left_wheel_revolutions -= revolutions
    right_wheel_revolutions += revolutions
    
    left_motor.setPosition(left_wheel_revolutions)
    right_motor.setPosition(right_wheel_revolutions)
    
    wait_turn()

# fucntion waits for turn to reach wanted degree
def wait_turn():
    while True:
        distance_left_now = left_position_sensor.getValue()
        distance_right_now = right_position_sensor.getValue()

        robot.step(timeStep)

        distance_left_later = left_position_sensor.getValue()
        distance_right_later = right_position_sensor.getValue()
        
        if (distance_left_now == distance_left_later) and (distance_right_now == distance_right_later):
            stop()
            break
        
# go forward distance in meters      
def forward(distance_m):  
    revolutions = distance_m / WHEEL_RADIUS
    
    left_wheel_revolutions = left_position_sensor.getValue()
    right_wheel_revolutions = right_position_sensor.getValue()

    left_wheel_revolutions += revolutions
    right_wheel_revolutions += revolutions

    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)

    left_motor.setPosition(left_wheel_revolutions)
    right_motor.setPosition(right_wheel_revolutions)
    
    PID_correction()
    
    if ds.getValue() > SENSOR_THRESHOLD + 45:
        front_correct()

# function to correct robot distance from left and right wall
def PID_correction():
    while True:
        distance_left_now = left_position_sensor.getValue()
        distance_right_now = right_position_sensor.getValue()
        
        left = ps[7].getValue()
        right = ps[0].getValue()
        left_wall = ps[5].getValue()
        right_wall = ps[2].getValue()
        right_angle_sensor = ps[1].getValue()
        left_angle_sensor = ps[6].getValue()

        previous_error = 0.00
        error_integral = 0.00
        P = 0.005
        I = 0.0008
        D = 0.0005
        Middle = 75
        
        if left_wall and right_wall:  
            error = left_angle_sensor - right_angle_sensor
            error_integral += error
            error_derivative = (previous_error - error)
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.2:
                MotorSpeed = 0.2
            elif MotorSpeed < -0.2:
                MotorSpeed = -0.2

            left_motor.setVelocity(MAX_SPEED + MotorSpeed)
            right_motor.setVelocity(MAX_SPEED - MotorSpeed)

        elif left_wall:
            error = left_angle_sensor - Middle           
            error_integral += error
            error_derivative = (previous_error - error)
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.06:
                MotorSpeed = 0.06
            elif MotorSpeed < -0.06:
                MotorSpeed = -0.06

            left_motor.setVelocity(MAX_SPEED + MotorSpeed)
            right_motor.setVelocity(MAX_SPEED - MotorSpeed)

        elif right_wall:
            error = right_angle_sensor - Middle

            error_integral += error
            error_derivative = (previous_error - error)
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.06:
                MotorSpeed = 0.06
            elif MotorSpeed < -0.06:
                MotorSpeed = -0.06

            left_motor.setVelocity(MAX_SPEED - MotorSpeed)
            right_motor.setVelocity(MAX_SPEED + MotorSpeed)
        
        robot.step(timeStep)
        
        distance_left_later = left_position_sensor.getValue()
        distance_right_later = right_position_sensor.getValue()

        if (distance_left_now == distance_left_later) and (distance_right_now == distance_right_later):
            stop()
            break

# function correct robot distance from front wall 
def front_correct():

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left = ps[7].getValue()
    right = ps[0].getValue()
    front = ds.getValue()
    
    desired_distance = SENSOR_THRESHOLD + 45
    
    if left < right:
        while left < right:
            left_motor.setVelocity(MAX_SPEED * 0.1)
            right_motor.setVelocity(MAX_SPEED * -0.1)
            robot.step(timeStep)
            left = ps[7].getValue()
            right = ps[0].getValue()

    elif left > right:
        while left > right:
            left_motor.setVelocity(MAX_SPEED * -0.1)
            right_motor.setVelocity(MAX_SPEED * 0.1)
            robot.step(timeStep)
            left = ps[7].getValue()
            right = ps[0].getValue()
    
    if front < desired_distance:
        while front < desired_distance:
            left_motor.setVelocity(MAX_SPEED * 0.1)
            right_motor.setVelocity(MAX_SPEED * 0.1)
            robot.step(timeStep)
            front = ds.getValue()
            
    elif front > desired_distance:
        while front > desired_distance:
            left_motor.setVelocity(MAX_SPEED * -0.1)
            right_motor.setVelocity(MAX_SPEED * -0.1)
            robot.step(timeStep)
            front = ds.getValue()
    stop()

# wall following maze solving
def wallfollowSolve():
    # sensor data
    left_wall = ps[5].getValue()
    right_wall = ps[2].getValue()
    left = ps[7].getValue()
    right = ps[0].getValue()
    front = ds.getValue()
    
    # location data in maze
    lw = left_wall > SENSOR_THRESHOLD
    rw = right_wall > SENSOR_THRESHOLD
    fw = front > SENSOR_THRESHOLD
    
    # wall following logic
    if not lw:
        updateVector()
        turn_left(90)
        updateVector()
        forward(UNIT_SIZE)
    else:
        if not fw:
            updateVector()
            forward(UNIT_SIZE)
        else:
            updateVector()
            turn_right(90)
    print(gs.getValue()) 
    # reached goal (dark floor)       
    if gs.getValue() < 400:
        updateVector()
        stop()
        return True
            
# main loop
while robot.step(timeStep) != -1:
    if wallfollowSolve():
        break
