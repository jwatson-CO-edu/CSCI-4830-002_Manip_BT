"""
FSM_ctrl.py
Finite State Machine that matches the C controller for the UR example world in WeBots
James Watson, 2020-02
"""

## NOTE, This file assumes the following:
## * All robots use this controller
## * There is at least robot named "UR5e"
## * UR5e rangefinder is named "range-finder"
## * UR5e camera      is named "camera"



########## INIT ##########

# Imports #
from controller import Robot, Motor, DistanceSensor, PositionSensor
from enum import Enum

# FSM States #
class State( Enum ):
    WAITING       = 0
    GRASPING      = 1
    ROTATING      = 2
    RELEASING     = 3
    ROTATING_BACK = 4 



########## WORLD ##########

# Constants #
TIME_STEP = 32

# create the Robot instance.
robot = Robot()
rName = robot.getName()
# get the time step of the current world.
timestep = int( robot.getBasicTimeStep() )
print( "Using timestep:", timestep )

##### Hand Actuators #####
hand_motors = []
hand_motors.append( robot.getDevice('finger_1_joint_1')      )
hand_motors.append( robot.getDevice('finger_2_joint_1')      )
hand_motors.append( robot.getDevice('finger_middle_joint_1') )

##### Arm Actuators #####
ur_motors = []
ur_motors.append( robot.getDevice('shoulder_lift_joint') )
ur_motors.append( robot.getDevice('elbow_joint')         )
ur_motors.append( robot.getDevice('wrist_1_joint')       )
ur_motors.append( robot.getDevice('wrist_2_joint')       )

##### Sensors #####
distance_sensor = robot.getDevice('distance sensor') 
distance_sensor.enable( TIME_STEP )
position_sensor = robot.getDevice('wrist_1_joint_sensor')
position_sensor.enable( TIME_STEP )
if rName == "UR5e":
    rangefnd_sensor = robot.getDevice('range-finder') 
    rangefnd_sensor.enable( TIME_STEP )
    rgbimage_sensor = robot.getDevice('camera') 
    rgbimage_sensor.enable( TIME_STEP )

print( "Robot controller init:", rName )



########## SIM SETTINGS ##########

dwellSteps = 10 # - Number of timesteps to pause
speed      =  1.2 # UR motor speed
for i, mtr in enumerate( ur_motors ):
    mtr.setVelocity( speed )
    
# Vars #
counter = 0
i       = 0
state   = State.WAITING

# Positions #
target_positions = [-1.88, -2.14, -2.38, -1.51]



########## MAIN LOOP ##########

# Main loop: perform simulation steps until Webots stops controller
while robot.step( timestep ) != -1:

    # If a state asked for a dwell time, no actions occur
    if counter > 0:
        counter -= 1
        continue

    # Read the sensors:
    dist = distance_sensor.getValue()
    posn = position_sensor.getValue()
    if rName == "UR5e":
        rang = rangefnd_sensor.getRangeImage()
        imag = rgbimage_sensor.getImage()

    # WAITING --> GRASPING #
    if state == State.WAITING:
        if dist < 500:
            state   = State.GRASPING
            counter = dwellSteps
            print( rName, ": Grasping can" )
            for mtr in hand_motors:
                mtr.setPosition( 0.85 )
    
    # GRASPING --> ROTATING #
    elif state == State.GRASPING:
        print( rName, ": Rotating arm" )
        state = State.ROTATING
        for i, mtr in enumerate( ur_motors ):
            mtr.setPosition( target_positions[i] )
    
    # ROTATING --> RELEASING #
    elif state == State.ROTATING:
        if posn < -2.3:
            counter = dwellSteps 
            state   = State.RELEASING
            print( rName, ": Releasing can" )
            for mtr in hand_motors:
                mtr.setPosition( mtr.getMinPosition() )
    
    # RELEASING --> ROTATING_BACK #
    elif state == State.RELEASING:
        state = State.ROTATING_BACK
        print( rName, ": Rotating arm back" )
        for i, mtr in enumerate( ur_motors ):
            mtr.setPosition( 0.0 )
    
    # ROTATING_BACK --> WAITING #
    elif state == State.ROTATING_BACK:
        if posn > -0.1:
            state = State.WAITING
            print( rName, ": Waiting can" );

    # !ERROR! #
    else:
        raise ValueError( str( rName ) + " : !BAD ROBOT STATE!" )
    
    # Decrement dwell counter
    counter -= 1

# Python GC performs cleanup
