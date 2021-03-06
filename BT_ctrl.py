"""
BT_ctrl.py
Behavior Tree that matches the C controller for the UR example world in WeBots
James Watson, 2020-03
"""

## NOTE, This file assumes the following:
## * All robots use this controller
## * There is at least robot named "UR5e"



########## INIT ####################################################################################

##### Imports #####
import sys
print( sys.version )

## Behavior Trees, https://py-trees.readthedocs.io/en/devel/introduction.html ##
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import FailureIsRunning
from py_trees.composites import Sequence
# from py_trees.behaviours import TickCounter
## WeBots ##
from controller import Robot, Motor, DistanceSensor, PositionSensor
## Other ##
from numpy import pi



##### UR Robots #################################

class UR_Controller( Robot ):
    """ Container class for URX Motors and Sensors """
    
    
    def __init__( self ):
        """ Connect to all motors and sensors """
        
        super().__init__()
        self.rName     = self.getName()
        self.TIME_STEP = 32
        
        self.dist = 0.0
        self.posn = 0.0
        self.dlay = 0
        
        
        ##### Hand Actuators #####
        self.hand_motors = []
        self.hand_motors.append( self.getDevice('finger_1_joint_1')      )
        self.hand_motors.append( self.getDevice('finger_2_joint_1')      )
        self.hand_motors.append( self.getDevice('finger_middle_joint_1') )

        ##### Arm Actuators #####
        self.ur_motors = []
        self.ur_motors.append( self.getDevice('shoulder_lift_joint') )
        self.ur_motors.append( self.getDevice('elbow_joint')         )
        self.ur_motors.append( self.getDevice('wrist_1_joint')       )
        self.ur_motors.append( self.getDevice('wrist_2_joint')       )

        ##### Sensors #####
        self.distance_sensor = self.getDevice('distance sensor') 
        self.distance_sensor.enable( self.TIME_STEP )
        self.position_sensor = self.getDevice('wrist_1_joint_sensor')
        self.position_sensor.enable( self.TIME_STEP )

        print( "Robot controller init:", self.rName )
        
        
    def set_motor_speed( self, speed = 1.2 ):
        """ Set the speed of all arm joint motors """
        for i, mtr in enumerate( self.ur_motors ):
            mtr.setVelocity( speed )
        
        
    def get_timestep( self ):
        """ get the time step of the current world """
        timestep = int( self.getBasicTimeStep() )
        print( "Using timestep:", timestep )
        return timestep
    
    
    def fetch_data( self ):
        """ Get distance and position """
        dist = self.distance_sensor.getValue()
        posn = self.position_sensor.getValue()
        return dist, posn
    
    
    def set_delay( self, delay = 10 ):
        """ Set the countdown clock """
        self.dlay = delay
        
        
    def q_delayed( self ):
        """ Return True if there is remaining countdown time """
        return (self.dlay > 0)
        
        
    def tick( self ):
        """ Decrement the countdown clock """
        if self.q_delayed():
            self.dlay -= 1
            

        
        
        
########## SIM SETTINGS #########################

# Robot #
rbt = UR_Controller()


# Constants #
dwellSteps = 10 # - Number of timesteps to pause



########## BEHAVIOR TREES ##########################################################################
"""
In this example, our needs to represent state are very simple, so state is stored in global variables only
`py_trees` offers a kind of advanced dictionary called a "Blackboard" that you can read about here:
https://py-trees.readthedocs.io/en/devel/blackboards.html
`import py_trees.blackboard as PTBlackboard`
"""

## State ##
d = 0.0 # Distance 
p = 0.0 # Position
c = 0 #-- Counter


""" ##### `py_trees` Behaviour Structure #####
In the py_trees library, Behaviors (Nodes) are represented by objects.
Behaviors you create should inherit `py_trees.behaviour.Behaviour` (Note UK spelling) so that all the
flow control mechanisms covered in class are inherited.
"""


class GetData( Behaviour ):
    """ Decrement the Counter and Fetch Data, 
    
    In order to function as expected within the BT framework, your custom behaviors should implement
    the following "template" functions, as needed
    """
    
    
    def __init__( self, name = "GetData", robotObj = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) # Sometimes, you may want instances of the same behavior class to 
        #                          have different names to help you with troubleshooting
        self.robot = robotObj
        
    
    def initialise( self ):
        """
        Run the first time your behaviour is ticked or not RUNNING. That is, this will be run again 
        if the  the behavior receives a tick after returning SUCCESS/FAILURE. 
        Expensive, long-running init should NOT be done here. ~ No return value
        """
        global d, p
        
        # If the counter is non-positive, we want to fetch data
        
        # 1. Fetch data
        d, p = self.robot.fetch_data()
    
    
    def update( self ):
        """
        Runs every time the behavior receives a tick.  Use this function to do or begin the work of
        this behavior.  The work inside this function should be relatively lightweight and 
        SHOULD (ideally) NOT BE BLOCKING (if possible). 
        ~ MUST return one of {RUNNING, SUCCESS, FAILURE, INVALID} from the `py_trees.common.Status` enum
        https://py-trees.readthedocs.io/en/devel/behaviours.html?highlight=status#status
        
        If the activity is long-running, it should be kicked off here, and immediately return with
        RUNNING to allow ticks to flow through the tree.  On successive ticks, this behavior must
        check the status of the long activity and report the appropriate status.
        """
        self.status = Status.SUCCESS
        
        
        # Always succeed
        return self.status
        
        
        
class TickCounter( Behaviour ):
    """
    This needed to be changed slightly from 
    https://py-trees.readthedocs.io/en/devel/modules.html?highlight=tickcounter#py_trees.behaviours.TickCounter
    """
    def __init__(
        self,
        duration          = 1,
        name              = "TickCounter",
        completion_status = Status.SUCCESS
    ):
        super().__init__( name = name )
        self.completion_status = completion_status
        self.duration          = duration
        self.counter           = 0
        self.lock              = 0

        
    def initialise( self ):
        """
        Reset the tick counter.
        """
        self.status = Status.RUNNING
        if not self.lock:
            self.lock = 1
            

    def update( self ):
        """
        Increment the tick counter and return the appropriate status for this behaviour
        based on the tick count.

        Returns
            :data:`~py_trees.common.Status.RUNNING` while not expired, the given completion status otherwise
        """
        self.counter += 1
        if self.counter <= self.duration:
            self.status = Status.RUNNING
        else:
            self.status  = self.completion_status
            self.lock    = 0
            self.counter = 0
        return self.status
        
        
        
""" ##### Conditions #####
Conditions are syntactically identical to Behaviors. By convention, a condition answers a question
with "True == SUCCESS" and "False == FAILURE", rather than taking an action.
"""
        
class COND_test_func( Behaviour ):
    """ Return SUCCESS if given function returns True, otherwise return FAIURE """
    
    def __init__( self, name = "COND_test_func", func = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) 
        self.func = func
        
    def initialise( self ):
        """ Remove INVALID status """
        self.status = Status.RUNNING
        
    def update( self ):
        """ Check parameter value against threshold """
        if self.func.__call__():
            self.status = Status.SUCCESS
        else:
            self.status = Status.FAILURE
        return self.status
        
        
        
##### Actions #####

class SetFingerAngles( Behaviour ):
    """ Send the finger motors to a configuration, pause """
    
    def __init__( self, name = "SetFingerAngles", robotObj = None, target = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) 
        self.robot  = robotObj
        self.target = target
        self.debug  = 0
    
    def initialise( self ):
        """ Set a target for the robot fingers, assume that they take `self.delay` timesteps to close """
        if self.debug:
            print( self.robot.rName, f", {self.name}: Moving fingers to target {self.target}" )
        for i, mtr in enumerate( self.robot.hand_motors ):
                mtr.setPosition( self.target[i] )
        self.status = Status.RUNNING
        
    def update( self ):
        """ Return 'RUNNING' until the counter runs out """
        self.status = Status.SUCCESS 
        return self.status
        
        
class SetArmAngles( Behaviour ):
    """ Send the arm motors to a configuration, no pause """
    
    def __init__( self, name = "SetArmAngles", robotObj = None, target = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) 
        self.robot  = robotObj
        self.target = target
        self.debug  = 0
        
    def initialise( self ):
        """ Set a target for the robot arm """
        if self.debug:
            print( self.robot.rName, f", {self.name}: Rotating arm to target {self.target}" )
        for i, mtr in enumerate( self.robot.ur_motors ):
            mtr.setPosition( self.target[i] )
        self.status = Status.RUNNING
            
    def update( self ):
        """ Always succeed """
        self.status = Status.SUCCESS
        return self.status
            

        
########## BUILD BT #############################
        

## Condition Functions ##
def d_LT_500():
    global d
    return d < 500

def p_LT_m2p3():
    global p
    return p < -2.3
    
def p_GT_m0p1():
    global p
    return p > -0.1

## Arm and Finger Configurations ##
graspFingers = [0.85 for _ in rbt.hand_motors]
relesFingers = [mtr.getMinPosition() for mtr in rbt.hand_motors]
turnArmConfg = [-1.88, -2.14, -2.38, -1.51]
backArmConfg = [0.0 for _ in rbt.ur_motors]

## Timing and Speed ##
if rbt.rName == "UR3e":
    dwell1 = 11
    dwell2 =  3
    dwell3 =  0
    rbt.set_motor_speed( 1.35 ) # UR motor speed
elif rbt.rName == "UR5e":
    dwell1 =  6
    dwell2 =  1
    dwell3 =  0
    rbt.set_motor_speed( 1.20 ) # UR motor speed
elif rbt.rName == "UR10e":
    dwell1 =  9
    dwell2 =  1
    dwell3 =  3
    rbt.set_motor_speed( 1.65 ) # UR motor speed

## Tree Structure ##

# Can collection subtree, has memory: Do not advance until prev child has completed #
actionSq = Sequence( memory = True )
actionSq.add_children([
    FailureIsRunning(  COND_test_func( "Grasp Dist Check", d_LT_500 )  ), # -- 1. WAIT for can to approach robot
    SetFingerAngles( "Grasp", rbt, graspFingers ), # ------------------------- 2. GRASP
    TickCounter( dwell1 ), # ------------------------------------------------- PAUSE for robot motors to move
    SetArmAngles( "Rotate", rbt, turnArmConfg ), # --------------------------- 3. ROTATE
    TickCounter( dwell2 ), # ------------------------------------------------- PAUSE for robot motors to move
    FailureIsRunning(  COND_test_func( "Release Posn Check", p_LT_m2p3 )  ), # 4. Do not release until at correct position
    SetFingerAngles( "Release", rbt, relesFingers ), # ----------------------- 5. RELEASE
    TickCounter( dwell3 ), # ------------------------------------------------- PAUSE for robot motors to move
    SetArmAngles( "Rotate Back", rbt, backArmConfg ), # ---------------------- 6. ROTATE BACK
])

# Root, no memory: tick all children every timestep #
rootNode = Sequence( memory = False )

# Every timestep we will
rootNode.add_children([
    GetData( "GetData", rbt ), # 1. Collect Data or Decrement Counter
    actionSq # ----------------- 2. Tick the can collection subtree
])



########## MAIN LOOP ###############################################################################

timestep = int( rbt.getBasicTimeStep() ) # Obtain timestep length
rootNode.setup_with_descendants() # ------ Get tree ready to execute
i =   0 # -------------------------------- Tick #
N = 300 # -------------------------------- N ticks between tree prints

# While the simulation is running
while rbt.step( timestep ) != -1:
    
    # Count Tick
    i += 1 
    
    # Send a tick down the tree from the root node
    rootNode.tick_once() 
    
    # Every N steps, Print the present state of the tree
    if i%N == 0:
        print( f"\n--------- Tick {i} ---------\n" ) # Print header
        print( f"Ran node: {rootNode.tip().name}" )  # Find and print the node that just run
        print( py_trees.display.unicode_tree(root=rootNode, show_status=True) )
        print("\n")