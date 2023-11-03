from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())

# constants
MAX_SPEED = 6.28
SENSOR_THRESHOLD = 100
left_speed = 0.0
right_speed = 0.0

# get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(left_speed)
right_motor.setVelocity(right_speed)

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

# delay time function   
def delay(ms):
    initTime = robot.getTime()
    while robot.step(timeStep) != -1:
        if (robot.getTime() - initTime) * 1000.0 > ms:
            break

# left wall following maze solving example (only works in perfect maze)
def wallfollowSolve():
    # get sensors values
    sensors_value = []
    for i in range(8):
        sensors_value.append(ps[i].getValue())
        
    # sensor data
    left_wall = sensors_value[5] > SENSOR_THRESHOLD
    left_corner = sensors_value[6] > SENSOR_THRESHOLD
    front_wall = (sensors_value[7]+sensors_value[0])/2 > SENSOR_THRESHOLD
    
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
            
    # Set motor speeds
    delay(5)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
#TODO: working bfs solve
def bfsSolve():
    # Set motor speeds
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
# main loop for robot functions
while robot.step(timeStep) != -1:
    wallfollowSolve()
    #bfsSolve()
