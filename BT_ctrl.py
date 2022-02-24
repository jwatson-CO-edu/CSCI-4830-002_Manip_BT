"""
BT_ctrl.py
Behavior Tree that matches the C controller for the UR example world in WeBots
James Watson, 2020-02
"""

## NOTE, This file assumes the following:
## * All robots use this controller
## * There is at least robot named "UR5e"
## * UR5e rangefinder is named "range-finder"
## * UR5e camera      is named "camera"



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
from py_trees.behaviours import TickCounter
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
        # if self.rName == "UR5e":
            # self.rangefnd_sensor = self.getDevice('range-finder') 
            # self.rangefnd_sensor.enable( self.TIME_STEP )
            # self.rgbimage_sensor = self.getDevice('camera') 
            # self.rgbimage_sensor.enable( self.TIME_STEP )

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
        # if self.rName == "UR5e":
            # rang = self.rangefnd_sensor.getRangeImage()
            # imag = self.rgbimage_sensor.getImage()
        return dist, posn
        
        
        
########## SIM SETTINGS #########################

# Robot #
rbt = UR_Controller()
rbt.set_motor_speed( 0.5 ) # UR motor speed

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
        global d, p, c
        
        # If the counter is non-positive, we want to fetch data
        
        # 1. Fetch data
        d, p = self.robot.fetch_data()
        
        c -= 1
    
    
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
        
    def update( self ):
        """ Check parameter value against threshold """
        if self.func():
            print( f"Condition \"{self.name}\" TRUE" )
            self.status = Status.SUCCESS
        else:
            print( f"Condition \"{self.name}\" FALSE" )
            self.status = Status.FAILURE
        return self.status
        
        
        
##### Actions #####

class SetFingerAngles( Behaviour ):
    """ Send the finger motors to a configuration, pause """
    
    def __init__( self, name = "SetFingerAngles", robotObj = None, delaySteps = 0, target = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) 
        self.robot  = robotObj
        self.delay  = delaySteps
        self.target = target
    
    def initialise( self ):
        """ Set a target for the robot fingers, assume that they take `self.delay` timesteps to close """
        global c
        
        if c < 0:
            print( self.robot.rName, f", {self.name}: Set fingers to {self.target}" )
            for i, mtr in enumerate( self.robot.hand_motors ):
                mtr.setPosition( self.target[i] )
            c = self.delay
        else:
            print( self.robot.rName, f", {self.name}: Counter OTHER than expected {c}" )
        # self.status = Status.RUNNING
        
    def update( self ):
        """ Return 'RUNNING' until the counter runs out """
        global c
        
        
        if c > 0:
            # self.status = Status.RUNNING
            return Status.RUNNING
        else:
            # self.status = Status.SUCCESS # The appropriate thing would be to actually *check* that goal position reached
            return Status.SUCCESS 
            

            
        return self.status
        
        
class SetArmAngles( Behaviour ):
    """ Send the arm motors to a configuration, no pause """
    
    def __init__( self, name = "SetArmAngles", robotObj = None, target = None ):
        """ Called once when the behavior is instantiated """
        super().__init__( name ) 
        self.robot  = robotObj
        self.target = target
        
    def initialise( self ):
        """ Set a target for the robot arm """
        print( self.robot.rName, f", {self.name}: Rotating arm to target {self.target}" )
        for i, mtr in enumerate( self.robot.ur_motors ):
            mtr.setPosition( self.target[i] )
        # self.status = Status.RUNNING
            
    def update( self ):
        """ Always succeed """
        # self.status = Status.SUCCESS
        # return self.status
        return Status.SUCCESS
            

        
########## BUILD BT #############################
        

## Condition Functions ##
def d_LT_500():
    global d
    return d < 500

def p_LT_m2p3():
    global p
    print( f"Release posn: {p}" )
    return p < -2.3
    
def p_GT_m0p1():
    global p
    print( f"Return posn: {p}" )
    return p > -0.1

## Arm and Finger Configurations ##
graspFingers = [0.85 for _ in rbt.hand_motors]
relesFingers = [mtr.getMinPosition() for mtr in rbt.hand_motors]
turnArmConfg = [-1.88, -2.14, -2.38, -1.51]
backArmConfg = [0.0 for _ in rbt.ur_motors]


## Tree Structure ##

# Can collection subtree, has memory: Do not advance until prev child has completed #
actionSq = Sequence( memory = True )
actionSq.add_children([
    # TickCounter( 1 ),
    FailureIsRunning(  COND_test_func( "Grasp Dist Check", d_LT_500 )  ), # - 1. WAIT for can to approach robot
    # TickCounter( 1 ),
    SetFingerAngles( "Grasp", rbt, dwellSteps, graspFingers ), # ------------ 2. GRASP
    # TickCounter( 1 ),
    SetArmAngles( "Rotate", rbt, turnArmConfg ), # -------------------------- 3. ROTATE
    # TickCounter( 1 ),
    FailureIsRunning(  COND_test_func( "Release Posn Check", p_LT_m2p3 )  ), # 4. Do not release until at correct position
    # TickCounter( 1 ),
    SetFingerAngles( "Release", rbt, dwellSteps*20 , relesFingers ), # ---------- 5. RELEASE
    # TickCounter( 1 ),
    SetArmAngles( "Rotate Back", rbt, backArmConfg ), # --------------------- 6. ROTATE BACK
    # TickCounter( 3 ),
    FailureIsRunning(  COND_test_func( "Return Posn Check", p_GT_m0p1 )  ), # 7. WAIT for arm to reach original config
    # TickCounter( 3 ),
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
i =   0 # --------------------------------- Iteration #
N = 100 # --------------------------------- N steps between tree prints

# While the simulation is running
while rbt.step( timestep ) != -1:
    
    # Count iteration
    i += 1 
    
    # Send a tick down the tree from the root node
    rootNode.tick_once() 
    
    print( f"Ran node: {rootNode.tip().name}" )
    
    # Every N steps, Print the present state of the tree
    if i%N == 0:
        print( f"\n--------- Tick {i} ---------\n" )
        print( py_trees.display.unicode_tree(root=rootNode, show_status=True) )
        print("\n")