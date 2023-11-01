from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# get a handler to the motors and set target position to infinity (speed control).
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# get a handler to the position sensors and enable them.
left_position_sensor = robot.getDevice('left wheel sensor')
right_position_sensor = robot.getDevice('right wheel sensor')
left_position_sensor.enable(timestep)
right_position_sensor.enable(timestep)

# constants
MAX_SPEED = 6.28
SENSORSENS = 120

# initialize distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
  
# main loop for robot functions
while robot.step(timestep) != -1:
    # get sensors values
    sensors_value = []
    for i in range(8):
        sensors_value.append(ps[i].getValue())
    
    # TODO: maze solving logic
    left_motor.setVelocity(MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
    