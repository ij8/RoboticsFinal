from klampt import *
from klampt import gldraw
from stateestimation import *
from OpenGL.GL import *

class MyController:
    """Attributes:
    - world: the WorldModel instance used for planning.
    - objectStateEstimator: a StateEstimator instance, which you may set up.
    - state: a string indicating the state of the state machine. TODO:
      decide what states you want in your state machine and how you want
      them to be named.  By default, this will go into the 'waiting' state
      on startup, and will go into the 'user' state when 'u' is pressed.

    By default, if 'u' is pressed, then this switches to 'user' mode,
    and uses the keys.
     - 1,2,3,4,5,6 to increase the robot's joint angles and
     - q,w,e,r,t,y to decrease the robot's joint angles.
    If 'u' is pressed again, it switches back to 'waiting' mode
    """
    def __init__(self,world,robotController):
        self.world = world
        self.objectStateEstimator = None
        self.state = None
        self.robotController = robotController
        self.reset(robotController)
        
    def reset(self,robotController):
        """Called on initialization, and when the simulator is reset.
        TODO: You may wish to fill this in with custom initialization code.
        """
        self.objectStateEstimator = MyObjectStateEstimator()
        self.objectEstimates = None
        self.state = 'waiting'
        #TODO: you may want to do more here to set up your
        #state machine and other initial settings of your controller.
        #The 'waiting' state is just a placeholder and you are free to
        #change it as you see fit.
        self.qdes = robotController.getCommandedConfig()
        # Initializing variable to contain list of previous object states
        # for displacement estimation. First element = oldest state
        self.prevWallPos = []
        self.prevWallVel = []
        self.prevWallAcc = []
        self.prevWallJer = []
        self.sensorBuffer = 0
        # Counter for number of tries the robot has made
        self.tries = 0
        # Global variable to set the number of tries
        self.maxTries = 10
        pass
    def checkCompleteConfig(self,robotController,nextState):
        count = 0 
        qsns = robotController.getSensedConfig()
        for i in range(0,len(self.qdes)):
            if round(self.qdes[i]*10**2) == round(qsns[i]*10**2):
                count += 1
        if count == len(self.qdes):
            self.state = nextState
        return count == len(self.qdes)
    def setRobotConfig(self,robotController,dt):
        robotController.setMilestone([0.0]*7,[0.0]*7)
        robotController.setCubic(self.qdes,[0.0]*7,dt)
    def getWallPos(self, objectStateEstimate):
        output = []
        for obj in objectStateEstimate.objects:
            if obj.meanPosition()[0] > .2 and obj.meanPosition()[2] > .2:
                output += [obj.meanPosition()[1]]
        return output
    def getWallDerivative(self, p1, p0, dt, n):
        output = []
        for i in range(0,len(p1)):
            output += [(p1[i]-p0[i])/(n*dt)]
        return output
    def checkBall(self,objectStateEstimate):
        return round(objectStateEstimate.objects[0].meanPosition()[0]*10) == round(-1*10) and round(objectStateEstimate.objects[0].meanPosition()[1]*10) == round(-.5*10)
    def myPlayerLogic(self,
                      dt,
                      sensorReadings,
                      objectStateEstimate,
                      robotController):
        """
        TODO: fill this out to updates the robot's low level controller
        in response to a new time step.  This is allowed to set any
        attributes of MyController that you wish, such as self.state.
        
        Arguments:
        - dt: the simulation time elapsed since the last call
        - sensorReadings: the sensor readings given on the current time step.
          this will be a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading
          coming from the omniscient object sensor.  You will not need to
          use raw sensor data directly, if you have a working state estimator.
        - objectStateEstimate: a MultiObjectStateEstimate class (see
          stateestimation.py) produced by the state estimator.
        - robotController: a SimRobotController instance giving access
          to the robot's low-level controller.  You can call any of the
          methods.  At the end of this call, you can either compute some
          PID command via robotController.setPIDCommand(), or compute a
          trajectory to execute via robotController.set/addMilestone().
          (if you are into masochism you can use robotController.setTorque())
        """
        #these are pulled out here for your convenience
        """
        qcmd = robotController.getCommandedConfig()
        vcmd = robotController.getCommandedVelocity()
        qsns = robotController.getSensedConfig()
        vsns = robotController.getSensedVelocity()
        """
        # Obtains the model for the robot       
        robot = self.world.robot(0)
        if self.state == 'waiting':
            """
            if self.tries == self.maxTries:
              self.state = 'done'
              pass
            """
            #TODO: do something..
            # Motion Queue Method for Batting
            self.qdes = [0,1.3,-2.37,-.8,1.5,.3,0]
            self.setRobotConfig(robotController,1)
            self.checkCompleteConfig(robotController,'sensing')
            self.prevWallPos = []
            self.prevWallVel = []
            self.prevWallAcc = []
            self.prevWallJer = []
            self.sensorBuffer = 0
        elif self.state == 'sensing':
            """ 
            Acquiring displacement estimate based on velocity, acceleration,
            and jerk estimates. Need to use the following equation:
            x(t) = x0 + v0t + a0t^2/2+j0t^3/6 + st^4/12
            where:  x0 is the initial position
                    v0 is the initial velocity
                    a0 is the initial acceleration
                    j0 is the initial jerk
                    s is the snap
            This requires using 5 points, so need to store 5 states in the
            list self.prevWallPos. Calculating up to snap should be
            sufficient in attaining reasonable position estimates
            """
            # Populate/update the historical position, velocity, and acceleration measurements
            if len(self.prevWallPos) < 5:
                self.prevWallPos += [self.getWallPos(objectStateEstimate)]
                if len(self.prevWallPos) > 1 and len(self.prevWallVel) < 4:
                    self.prevWallVel += [self.getWallDerivative(self.prevWallPos[-1],self.prevWallPos[-2],dt,1)]
                    if len(self.prevWallVel) > 1 and len(self.prevWallAcc) < 3:
                        self.prevWallAcc += [self.getWallDerivative(self.prevWallVel[-1],self.prevWallVel[-2],dt,2)]
                        if len(self.prevWallAcc) > 1 and len(self.prevWallJer) < 2:
                            self.prevWallJer += [self.getWallDerivative(self.prevWallPos[-1],self.prevWallPos[-2],dt,3)]
            # Buffer initial readings to weed out outliers from the first few readings
            elif self.sensorBuffer < 15:
                self.prevWallPos.pop(0)
                self.prevWallVel.pop(0)
                self.prevWallAcc.pop(0)
                self.prevWallJer.pop(0)
                self.sensorBuffer += 1
            else:
                # Predict positions of each wall
                currentWallSnap = self.getWallDerivative(self.prevWallAcc[-1],self.prevWallAcc[-2],dt,4)
                predictedMeanPos = []
                for i in range(0,len(currentWallSnap)):
                    # Assumption that it takes 2.5 seconds to reach 
                    t = 1.5
                    x0 = self.prevWallPos[-1][i]
                    v0 = self.prevWallVel[-1][i]
                    a0 = self.prevWallAcc[-1][i]
                    j0 = self.prevWallJer[-1][i]
                    s = currentWallSnap[i]
                    predictedMeanPos += [x0 + v0*t + a0*t**2/2 + j0*t**3/6 + s*t**4/12]
                print predictedMeanPos
                self.prevWallPos.pop(0)
                self.prevWallVel.pop(0)
                self.prevWallAcc.pop(0)
                self.prevWallJer.pop(0)
                # Iterate through points in goal and find best target
                target = -1
                res = 100
                maxDiff = 0
                minDist = 1.5
                for i in range(1,res+1):
                    # Total goal width approx 2, resoltuion 100
                    currTarget = 1.0 - 2.0*float(i)/float(res)
                    # Count number of obstacles outside target window
                    count = 0
                    diffs = []
                    for j in range(0,len(predictedMeanPos)):
                        if abs(predictedMeanPos[j]-currTarget) > minDist:
                            count += 1
                            diffs += [abs(predictedMeanPos[j]-currTarget)]
                    # Update the target if it's open
                    # Note: In terms of res unit (easier to convert to
                    # proper angle adjustment)
                    if count == len(predictedMeanPos) and min(diffs) > maxDiff:
                        maxDiff = min(diffs)
                        target = i
                # If a target is open and the ball is in its spawning 
                # position, then set the angle (-.1 = rightmost, .8 = leftmost)
                # and strike (also update preObjectState)
                if target != -1 and self.checkBall(objectStateEstimate):
                    self.qdes[5] = 1 - 1.2*float(target)/float(res)
                    print '--------------------'
                    print 'target coord:'
                    print 1.0 - 2.0*float(i)/float(res)
                    print 'target:'
                    print target
                    print 'angle:'
                    print self.qdes[5]
                    print 'Prev'
                    print self.prevWallPos[-1]
                    print 'predicted'
                    print predictedMeanPos
                    print '--------------------'
                    self.state = 'strike'   
        elif self.state == 'strike':
            # Motion Queue Method for Striking
            self.qdes[1] = 1.8
            self.setRobotConfig(robotController,.22)
            if self.checkCompleteConfig(robotController,'reverting'):
                self.tries += 1
        elif self.state == 'reverting':
            # Motion Queue Method for Striking
            self.qdes[1] = 1.6
            self.qdes[2] = -2.2
            self.setRobotConfig(robotController,.2)
            self.checkCompleteConfig(robotController,'waiting')
        elif self.state == 'done':
            print 'done'
            pass
        elif self.state == 'user':
            #use the user-mode control
            robotController.setPIDCommand(self.qdes,[0.0]*7)
        else:
            #TODO: do something else...
            #may want to add other states into this if block...
            pass
        return
    def loop(self,dt,robotController,sensorReadings):
        """Called every control loop (every dt seconds).
        Input:
        - dt: the simulation time elapsed since the last call
        - robotController: a SimRobotController instance. Use this to get
          sensor data, like the commanded and sensed configurations.
        - sensorReadings: a dictionary mapping sensor names to sensor data.
          The name "blobdetector" indicates a sensor reading coming from the
          blob detector.  The name "omniscient" indicates a sensor reading coming
          from the omniscient object sensor.
        Output: None.  However, you should produce a command sent to
          robotController, e.g., robotController.setPIDCommand(qdesired).

        """
        multiObjectStateEstimate = None
        if self.objectStateEstimator and 'blobdetector' in sensorReadings:
            multiObjectStateEstimate = self.objectStateEstimator.update(sensorReadings['blobdetector'])
            self.objectEstimates = multiObjectStateEstimate
            #multiObjectStateEstimate is now a MultiObjectStateEstimate (see stateestimator.py)
        if 'omniscient' in sensorReadings:
            omniscientObjectState = OmniscientStateEstimator().update(sensorReadings['omniscient'])
            #omniscientObjectStateEstimate is now a MultiObjectStateEstimate (see stateestimator.py)
            
            #TODO: Comment out the following line when you are ready to test your state estimator
            multiObjectStateEstimate  = omniscientObjectState

        self.myPlayerLogic(dt,
                           sensorReadings,multiObjectStateEstimate,
                           robotController)
        return

    def keypress(self,key):
        """If you want to implement some interactivity while debugging,
        you can do it here. By default, it uses 1,2,3,4,5,6 to increase
        the robot's joint angles and q,w,e,r,t,y to to decrease them.
        """
        if key == 'u':
            if self.state == 'user':
                print "Switching out of user mode..."
                self.state = 'waiting'
            else:
                print "Switching into user mode..."
                self.state = 'user'
                self.qdes = self.robotController.getCommandedConfig()
        if self.state == 'user':
            #note: joint 0 is a dummy joint
            upkeys = {'1':1,'2':2,'3':3,'4':4,'5':5,'6':6}
            downkeys = {'q':1,'w':2,'e':3,'r':4,'t':5,'y':6}
            if key in upkeys:
                self.qdes[upkeys[key]] += 0.1
            elif key in downkeys:
                self.qdes[downkeys[key]] -= 0.1
    
    def drawGL(self):
        """This gets called every time an OpenGL rendering loop is called.
        TODO: You may consider visually debugging some of your code here.

        For example, to draw the robot at a given configuration q, you can call:
          self.world.robot(0).setConfig(q)
          self.world.robot(0).drawGL()

        To draw a point with size s, color (r,g,b), and world position (x,y,z)
        you can call:
          glDisable(GL_LIGHTING)
          glColor3f(r,g,b)
          glPointSize(s)
          gldraw.point([x,y,z])

        The current code draws gravity-inflenced arcs leading from all the
        object position / velocity estimates from your state estimator.  Event C
        folks should set gravity=0 in the following code.
        """
        glDisable(GL_LIGHTING)
        glColor3f(100,100,100)
        glPointSize(5.0)
        gldraw.point([0,0,0])
        gldraw.point([1,0,0])
        gldraw.point([0,1,0])
        gldraw.point([0,0,1])
        gldraw.point([2.025,0,0])
        # Camera Location
        gldraw.point([-1.5,-.5,.25])
        if self.objectEstimates:
            for o in self.objectEstimates.objects:
                glDisable(GL_LIGHTING)
                glColor3f(o.name[0],o.name[1],o.name[2])
                #draw a point
                glPointSize(5.0)
                gldraw.point([o.x[0],o.x[1],o.x[2]])
                #draw an arc
                glBegin(GL_LINE_STRIP)
                x = [o.x[0],o.x[1],o.x[2]]
                v = [o.x[3],o.x[4],o.x[5]]
                #TODO: are you doing event C? If so, you should
                #set gravity=0 to get more useful visual feedback
                #about your state estimates.
                #gravity = 0
                gravity = 9.8
                for i in range(20):
                    t = i*0.05
                    glVertex3f(*vectorops.sub(vectorops.madd(x,v,t),[0,0,0.5*gravity*t*t]))
                glEnd()
