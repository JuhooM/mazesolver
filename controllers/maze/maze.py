from controller import Robot
import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())

# constants
MAX_SPEED = 6.28 # maximum speed of robot
SENSOR_THRESHOLD = 80 # sensitivity of sensors surrounding robot for obstacle detection
UNIT_SIZE = 0.125 # size of 1 maze node in mmeters
WHEEL_RADIUS = 0.0205 # radius of robots wheel in mmeters
AXLE_LENGTH = 0.052 # lenght of the robots axle of wheels in mmeters

# variables
vel = [0.0, 0.0] # wheel speed [left wheel speed, right wheel speed]
pos = [0.0, 0.0] # wheel position changed in rad [left wheel pos, right wheel pos]
dis = [0.0, 0.0] # wheel distance traveled in meters [left wheel dis, right wheel dis]
dirvec = [0.0, 0.0, 0.0] # position vector of robot [x axis position(robot face at start)-meter, y axis position-meter, orientation-rad]

# get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(vel[0])
right_motor.setVelocity(vel[1])

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
        pos[0] = left_position_sensor.getValue()
        pos[1] = right_position_sensor.getValue()
        
        # calculate distance travelled from previous values
        distance1 = abs(pos[0] * WHEEL_RADIUS - dis[0])
        distance2 = abs(pos[1] * WHEEL_RADIUS - dis[1])
        avgdist = (distance1+distance2)/2

        # update distance travelled in meters
        dis[0] = pos[0] * WHEEL_RADIUS
        dis[1] = pos[1] * WHEEL_RADIUS
        
        # update dirvec
        if not turn:
            dirvec[0] += avgdist * math.cos(dirvec[2])
            dirvec[1] += avgdist * math.sin(dirvec[2])
        else:
            dirvec[2] += (2*avgdist)/AXLE_LENGTH;
        
        print('Robots position [x-axis, y-axis, orientation]')
        print(dirvec)
        print(' ')
        
        return avgdist
    
# sets robots speed to match vel values
def setSpeed():
    left_motor.setVelocity(vel[0])
    right_motor.setVelocity(vel[1])

# turn right turn_amount degrees
def turn_right(turn_amount_deg):
    stop()
    # set left wheel speed
    vel[0] = MAX_SPEED/10
    # set right wheel speed
    vel[1] = -MAX_SPEED/10
    setSpeed()

    # turn right wanted amount
    goal = turn_amount_deg/360 * 2 * math.pi + dirvec[2]
    travelled = 0
    while robot.step(timeStep) != -1:
        travelled += disTravelled(True)
        if pow(goal-dirvec[2], 2) < 0.00001:
            break

# turn left turn_amount degrees
def turn_left(turn_amount_deg):
    stop()
    # set left wheel speed
    vel[0] = -MAX_SPEED/10
    # set right wheel speed
    vel[1] = MAX_SPEED/10
    setSpeed()

    # turn left wanted amount
    goal = turn_amount_deg/360 * 2 * math.pi + dirvec[2]
    travelled = 0
    while robot.step(timeStep) != -1:
        travelled += disTravelled(True)
        if pow(goal-dirvec[2], 2) < 0.00001:
            break

# go forward distance in meters
def forward(distance_m):
    stop()
    # set left wheel speed
    vel[0] = MAX_SPEED
    # set right wheel speed
    vel[1] = MAX_SPEED
    setSpeed()
    
    # go forward wanted distance
    goal = distance_m
    travelled = 0
    while robot.step(timeStep) != -1:
            travelled += disTravelled(False)
            if pow(goal-travelled, 2) < 0.00001:
                break
            
# stop robot movement function
def stop():
    #set left wheel speed
    vel[0] = 0
    #set right wheel speed
    vel[1] = 0
    setSpeed()

# delay time function for delay_ms
def delay(delay_ms):
    initTime = robot.getTime()
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > delay_ms:
            break

# better wall following maze solving
def betterwallfollowSolve():
    # get sensors values
    sensors_value = []
    for i in range(8):
        sensors_value.append(ps[i].getValue())
        
    # sensor data
    left = sensors_value[5]
    right = sensors_value[2]
    front = (sensors_value[7]+sensors_value[0])/2
    
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
            
# left wall following maze solving example, only works in perfect maze and gets stuck sometimes
def wallfollowSolve():
    # get sensors values
    sensors_value = []
    for i in range(8):
        sensors_value.append(ps[i].getValue())
        
    # sensor data
    left_wall = sensors_value[5] > SENSOR_THRESHOLD+20
    left_corner = sensors_value[6] > SENSOR_THRESHOLD
    front_wall = (sensors_value[7]+sensors_value[0])/2 > SENSOR_THRESHOLD+20
    
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
    delay(10)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

# main loop for robot functions
while robot.step(timeStep) != -1:
    #wallfollowSolve()
    betterwallfollowSolve()
