from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())

# constants
MAX_SPEED = 4 # speed of robot
SENSOR_THRESHOLD = 80 # sensitivity of sensors surrounding robot for obstacle detection
UNIT_SIZE = 0.125 # size of 1 maze node in mmeters
WHEEL_RADIUS = 0.02002 # radius of robots wheel in mmeters
AXLE_LENGTH = 0.0568 # lenght of the robots axle of wheels in mmeters

# variables
pos = [0.0, 0.0] # wheel position changed in rad [left wheel pos, right wheel pos]
dis = [0.0, 0.0] # wheel distance traveled in meters [left wheel dis, right wheel dis]
dirvec = [0.0, 0.0, 0.0] # position vector of robot [x axis position(robot face at start)-meter, y axis position-meter, orientation-rad]

# get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# get a handler to the position sensors and enable them.
left_position_sensor = robot.getDevice('left wheel sensor')
right_position_sensor = robot.getDevice('right wheel sensor')
left_position_sensor.enable(timeStep)
right_position_sensor.enable(timeStep)

# initialize distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timeStep)

# return distance travelled of wheels from previous function call to current time, also update dirvec, par turn true if you are turning otherwise false
def disTravelled(turn: bool):
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
        if turn:
            #if distance1 < 0 or distance2 < 0:
            if distance1 < 0 and distance2 > 0:
                dirvec[2] += (abs(distance1) + abs(distance2))/AXLE_LENGTH;
            elif distance1 > 0 and distance2 < 0:
                dirvec[2] -= (abs(distance1) + abs(distance2))/AXLE_LENGTH;
        else:
            dirvec[0] += avgdist * math.cos(dirvec[2])
            dirvec[1] += avgdist * math.sin(dirvec[2])
        
        return avgdist

# turn right turn_amount degrees
def turn_right(turn_amount_deg):
    goal = dirvec[2] - turn_amount_deg/360 * 2 * math.pi
    # set left wheel speed
    left_motor.setVelocity(MAX_SPEED*0.33)
    # set right wheel speed
    right_motor.setVelocity(-MAX_SPEED*0.33)
    
    # turn right wanted amount
    while True:
        disTravelled(True)
        robot.step(timeStep)
        if pow(goal-dirvec[2], 2) < 0.001:
            break

# turn left turn_amount degrees
def turn_left(turn_amount_deg):
    goal = dirvec[2] + turn_amount_deg/360 * 2 * math.pi
    # set left wheel speed
    left_motor.setVelocity(-MAX_SPEED*0.33)
    # set right wheel speed
    right_motor.setVelocity(MAX_SPEED*0.33)

    # turn left wanted amount
    while True:
        disTravelled(True)
        robot.step(timeStep)
        if pow(goal-dirvec[2], 2) < 0.001:
            break

# go forward distance in meters
def forward(distance_m):
    goal = distance_m
    travelled = 0
    # set left wheel speed
    left_motor.setVelocity(MAX_SPEED)
    # set right wheel speed
    right_motor.setVelocity(MAX_SPEED)
    # go forward wanted distance
    while True:
        travelled += disTravelled(False)
        robot.step(timeStep)
        
        PID_correction()
        
        left = ps[7].getValue()
        right = ps[0].getValue()
        if pow(goal-travelled, 2) < 0.00008 or (left + right)/2 >= SENSOR_THRESHOLD:
            break
    stop()
    delay(100)
    
    if (right + left) / 2 > SENSOR_THRESHOLD:
        front_correct(left, right)
        
def bforward(distance_m):
    revolutions = maze_parameters.TILE_LENGTH / robot_parameters.WHEEL #rev in radians
    
    left_wheel_revolutions = ps_left.getValue()
    right_wheel_revolutions = ps_right.getValue()

    left_wheel_revolutions += revolutions
    right_wheel_revolutions += revolutions

    left_motor.setVelocity(robot_parameters.SPEED)
    right_motor.setVelocity(robot_parameters.SPEED)

    left_motor.setPosition(left_wheel_revolutions)
    right_motor.setPosition(right_wheel_revolutions)
              
# stop robot movement function
def stop():
    #set left wheel speed
    left_motor.setVelocity(0)
    #set right wheel speed
    right_motor.setVelocity(0)

# delay time function for delay_ms
def delay(delay_ms):
    initTime = robot.getTime()
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > delay_ms:
            break

# function to correct robot distance from left and right wall
def PID_correction():
    while True:
        distance_left_now = left_position_sensor.getValue()
        distance_right_now = right_position_sensor.getValue()

        right_angle_sensor = ps[1].getValue()
        left_angle_sensor = ps[6].getValue()
        left_wall = ps[5].getValue()
        right_wall = ps[2].getValue()
        
        left = ps[7].getValue()
        right = ps[0].getValue()

        previous_error = 0.00
        error_integral = 0.00
        P = 0.005  #0.005
        I = 0.0008 #0.0005  0.0001
        D = 0.0005 # 0.0002
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
    
        distance_left_later = left_position_sensor.getValue()
        distance_right_later = right_position_sensor.getValue()

        if (distance_left_now == distance_left_later) and (distance_right_now == distance_right_later):
            break

# function correct robot distance from front wall 
def front_correct(left, right):
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    front = (left + right) / 2
    desired_distance = SENSOR_THRESHOLD + 20

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
            left = ps[7].getValue()
            right = ps[0].getValue()
            front = (left + right) / 2

    elif front > desired_distance:
        while front > desired_distance:
            left_motor.setVelocity(MAX_SPEED * -0.1)
            right_motor.setVelocity(MAX_SPEED * -0.1)
            robot.step(timeStep)
            left = ps[7].getValue()
            right = ps[0].getValue()
            front = (left + right) / 2

# better wall following maze solving
def betterwallfollowSolve():
    # sensor data
    left = ps[5].getValue()
    right = ps[2].getValue()
    front = (ps[7].getValue() + ps[0].getValue())/2
    
    # location data in maze
    left_wall = left > SENSOR_THRESHOLD
    right_wall = right > SENSOR_THRESHOLD
    front_wall = front > SENSOR_THRESHOLD
    
    # wall following logic
    if not left_wall:
        turn_left(90)
        forward(UNIT_SIZE)
    else:
        if not front_wall:
            forward(UNIT_SIZE)
        else:
            turn_right(90)
    
    #stop()
    #delay(100)   
    #if dirvec[0] >= 0.35 and dirvec[0] <= 0.625 and dirvec[1] >= 0.35 and dirvec[1] <= 0.625:
        #return True
            
# left wall following maze solving example, only works in perfect maze and gets stuck sometimes
def wallfollowSolve():        
    # sensor data
    left_wall = ps[5].getValue() > SENSOR_THRESHOLD+20
    left_corner = ps[6].getValue() > SENSOR_THRESHOLD
    front_wall = (ps[7].getValue() + ps[0].getValue())/2 > SENSOR_THRESHOLD+40
    
    # intial speed
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # wall following logic
    if front_wall:
        left_speed = MAX_SPEED
        right_speed = -MAX_SPEED
    else:
        if left_wall:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED/8
            right_speed = MAX_SPEED
        if left_corner:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED/6

    # Set motor vel
    delay(50)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# main loop
while robot.step(timeStep) != -1:
    #wallfollowSolve():
    if betterwallfollowSolve():
        break
