#!/usr/bin/env python3
#
#   Create a motion by continually sending joint values.  Also listen
#   to the point input.
#
#   Publish:   /joint_states      sensor_msgs/JointState
#   Subscribe: /point             geometry_msgs/PointStamped
#              /start_check       std_msgs/Bool
#
import rospy
import numpy as np
import math
import time
import copy

from fruitninja.kinematics2 import Kinematics
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


#
#  Point Subscriber
#
#  This listens for and saves the value of the last received point
#  message.  It isolate the ROS message subscriber to keep the main
#  code simpler.
#
class PointSubscriber:
    def __init__(self):
        # Instantiate the point p as a numpy vector.
        self.p = np.array([[0.0], [0.0], [0.0]])

        # Mark no arrived data.
        self.arrived = False

        # Create a subscriber to listen to point messages.
        rospy.Subscriber("point", PointStamped, self.process)

    def valid(self):
        return self.arrived

    def position(self):
        # Return the point.
        return self.p

    def process(self, msg):
        # Save the point contained in the message.
        self.p[0] = msg.point.x
        self.p[1] = msg.point.y
        self.p[2] = msg.point.z

        # Mark as valid.
        self.arrived = True


class StartSubscriber:
    def __init__(self):
        self.state = False

        # Create a subscriber to listen to the start button command
        rospy.Subscriber("start_check", Bool, self.process)

    def start(self):
        # Return the state
        return self.state

    def process(self, msg):
        # Check if sim has started

        self.state = msg.data


#
#  Joint States Publisher
#
#  Isolate the ROS message publisher to keep the main code simpler.
#
class JointStatePublisher:
    def __init__(self, names):
        # Save the dofs = number of names.
        self.n = len(names)

        # Create a publisher to send the joint values (joint_states).
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=100)

        # Wait until connected.  You don't have to wait, but the first
        # messages might go out before the connection and hence be lost.
        rospy.sleep(0.25)

        # Create a joint state message.
        self.msg = JointState()

        # You have to explicitly name each joint: Keep appending to
        # create a list.
        for i in range(self.n):
            self.msg.name.append(names[i])

        # We should also prepare the position list, initialize to zero.
        for i in range(self.n):
            self.msg.position.append(0.0)

        # Report.
        rospy.loginfo("Ready to publish /joint_states with %d DOFs", self.n)

    def dofs(self):
        # Return the number of DOFs.
        return self.n

    def send(self, q):
        # Set the positions.
        for i in range(self.n):
            self.msg.position[i] = q[i]

        # Send the command (with specified time).
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)


#
#  Basic Rotation Matrices
#
#  Note the angle is specified in radians.
#
def Rx(phi):
    return np.array([[1, 0, 0],
                     [0, np.cos(phi), -np.sin(phi)],
                     [0, np.sin(phi), np.cos(phi)]])


def Ry(phi):
    return np.array([[np.cos(phi), 0, np.sin(phi)],
                     [0, 1, 0],
                     [-np.sin(phi), 0, np.cos(phi)]])


def Rz(phi):
    return np.array([[np.cos(phi), -np.sin(phi), 0],
                     [np.sin(phi), np.cos(phi), 0],
                     [0, 0, 1]])


#
#  Simple Vector Utilities
#
#  Just collect a 3x1 column vector, perform a dot product, or a cross product.
#
def vec(x, y, z):
    return np.array([[x], [y], [z]], dtype='float')


def dot(a, b):
    return a.T @ b


def cross(a, b):
    return np.cross(a, b, axis=0)


# following two functions are used if a very specific motion is desired for sword
# no use if letting algorithm determine final cutting direction


#  6x1 Error Computation
#
#  Note the 3x1 translation is on TOP of the 3x1 rotation error!
#
#  Also note, the cross product does not return a column vector but
#  just a one dimensional array.  So we need to convert into a 2
#  dimensional matrix, and transpose into the column form.  And then
#  we use vstack to stack vertically...
#

def etip(p, pd, R, Rd):
    ep = pd - p
    eR = 0.5 * (cross(R[:, [0]], Rd[:, [0]]) +
                cross(R[:, [1]], Rd[:, [1]]) +
                cross(R[:, [2]], Rd[:, [2]]))
    return np.vstack((ep, eR))


# produce a rotation matrix to align two unit vectors
def vecAlign(vec1, vec2):
    axis = cross(vec1, vec2)  # axis to rotate -x axis of tip onto vector
    cosvec = dot(vec1, vec2)  # cosine of angle between two vectors
    skewsym = np.array([[0, -axis[2,0], axis[1,0]], [axis[2,0], 0, -axis[0,0]],
                        [-axis[1,0], axis[0,0], 0]], dtype='float')  # skew symmetric matrix of axis
    desrot = np.add(np.identity(3), np.add(skewsym, skewsym @ skewsym * 1 / (1 + cosvec)))  # rotation matrix to rotate
    return desrot, axis


#
#  Main Code
#
def setup():
    #
    #  LOGISTICAL SETUP
    #
    # Prepare the node.
    rospy.init_node('fruit_ninja_ikin')
    rospy.loginfo("Starting Code for Fruit Ninja")

    # Prepare a servo loop at 100Hz.
    rate = 100;
    servo = rospy.Rate(rate)
    dt = servo.sleep_dur.to_sec()
    rospy.loginfo("Running with a loop dt of %f seconds (%fHz)" %
                  (dt, rate))

    # Set up the kinematics, from world to tip.
    urdf = rospy.get_param('/robot_description')
    kin = Kinematics(urdf, 'world', 'tip')
    N = kin.dofs()
    rospy.loginfo("Loaded URDF for %d joints" % N)

    # Set up the publisher, naming the joints!
    pub = JointStatePublisher(('theta1', 'theta2', 'theta3',
                               'theta4', 'theta5', 'theta6', 'theta7'))

    time.sleep(.3)
    theta = np.array([[0.0], [-0.5], [0.0], [1.0], [0.0], [0.0], [1.0]]) # start elbow down in a natural sword holding position
    pub.send(theta)

    # Make sure the URDF and publisher agree in dimensions.
    if not pub.dofs() == kin.dofs():
        rospy.logerr("FIX Publisher to agree with URDF!")

    # Set up the point subscriber.
    point = PointSubscriber()
    started = StartSubscriber()
    rospy.loginfo("Waiting for a point...")

    while not rospy.is_shutdown() and not point.valid() or not started.start():  # wait for valid point and for start
        pass
    rospy.loginfo("Got a point.")
    cs1 = np.zeros((7, 4))  # spline 1 coefficient matrix
    cs2 = np.zeros((7, 4))  # spline 2 coefficient matrix

    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress=True, precision=6)

    # Our robot moves from a generic elbow down position to the slash position, so this will have good initial guess
    theta = np.array([[0.0], [-0.5], [0.0], [1.0], [0.0], [0.0], [1.0]])
    pub.send(theta)

    thetastart = copy.deepcopy(theta) # keep copy of starting position to go back to in 2nd spline

    # get position and desired velocity of input point
    pos = point.position()

    thetaprev = np.array([[10.0], [10.0], [10.0], [10.0], [10.0], [10.0], [10.0]])  # use for iterating newton-raphson
    thetadot = np.zeros((7,1))
    
    thetamid = copy.deepcopy(theta)
    thetamid[0,0] = (np.arctan2(-pos[0,0],pos[1,0])+ np.pi/4*np.sign(np.arctan2(-pos[0,0],pos[1,0])) + np.pi) % (2 * np.pi) - np.pi # wraps joint angle to pi
    
    
    lam = 0.1/dt


    # perform newton raphson iteration algorithm on final 
    while abs(np.linalg.norm(theta) - np.linalg.norm(thetaprev)) > .01:
        (p, R) = kin.fkin(theta)
        J = kin.Jac(theta)[0:3,:] # position component of jacobian
        Jinv = np.linalg.pinv(J) 
        theta0 = theta
        theta = theta0 + Jinv @ np.subtract(pos,p) # take error based on positional error
        thetaprev = theta0


    theta = (theta + np.pi) % (2 * np.pi) - np.pi # wrap joint angles to +/- pi
    J = kin.Jac(theta)
    Jinv = np.linalg.pinv(J)
    vel = R @ vec(1,0,0) # align cutting edge (x-axis) to the final rotation of the blade

    vd = np.vstack((vel,vec(0,0,0))) # want it to not be rotating as it cuts the fruit
    thetadot = Jinv @ vd
    thetadot = thetadot / np.linalg.norm(thetadot) * 10

    thetacur = np.zeros((7,1))
    
    #tstart = -1
    t = 0 #tstart
    t0 = 0
    tslice = 2
    tf = 7
    for i in range(7):
        s1sys = np.array([[1, 0, 0, 0], \
                               [0, 1, 0, 0], \
                               [1, tslice - t0, (tslice - t0) ** 2, (tslice - t0) ** 3], \
                               [0, 1, 2*(tslice - t), 3*(tslice - t) ** 2]], dtype='float')  # system of equations for spline 1

        s1val = np.array([thetastart[i,0], 0, theta[i,0], thetadot[i,0]], dtype='float')  # matrix of p0, v0, pf, vf for spline 1
        cs1[i,:] = np.linalg.solve(s1sys, s1val) # set coefficient matrix 1 to the solution to system of equations as specified above
        s2sys = np.array([[1, 0, 0, 0], \
                               [0, 1, 0, 0], \
                               [1, tf - tslice, (tf - tslice) ** 2, (tf - tslice) ** 3], \
                               [0, 1, 2*(tf - tslice), 3*(tf - tslice) ** 2]], dtype='float')  # system of equations for spline 1

        s2val = np.array([theta[i,0], thetadot[i,0], thetastart[i,0], 0], dtype='float')  # matrix of p0, v0, pf, vf for spline 1

        cs2[i,:] = np.linalg.solve(s2sys, s2val) # set coefficient matrix 2 to the solution to system of equations as specified above


        #
        #  TIME LOOP
        #
        #
    lam = 0.1 / dt # error term

    while not rospy.is_shutdown():
        # checks if simulation has been started by button on gui
        if started.state:
            # Using the result theta(i-1) of the last cycle (i-1):
            # Compute the forward kinematics and the Jacobian.

            # Advance to the new time step.
            t += dt
            for i in range(7):
                #if t< 0: # before t=0, move to a good starting position to help alleviate clipping
                    #thetacur[i,0] = thetastart[i,0] + (thetamid[i,0]-thetastart[i,0])/(1)*((t-tstart)-1/(np.pi*2)*np.sin(2*np.pi*(t-tstart)/1))
                if 0<t<tslice:
                    thetacur[i,0]=cs1[i,0]+cs1[i,1]*t+cs1[i,2]*t**2+cs1[i,3]*t**3 # spine 1 from t=0 to slicing the fruit
                elif tslice<=t<=tf:
                    thetacur[i,0]=cs2[i,0]+cs2[i,1]*(t-tslice)+cs2[i,2]*(t-tslice)**2+cs2[i,3]*(t-tslice)**3 # spine 2 from t=0 to slicing the fruit
                #elif tf < t < tf+1:
                    #thetacur[i,0] = thetamid[i,0] + (thetastart[i,0]-thetamid[i,0])/(1)*((t-tf)-.5/(np.pi*2)*np.sin(2*np.pi*(t-tf)/1))
                else:
                    thetacur[i,0]=thetastart[i,0] # stay at starting position until reset
                

            # Publish and sleep for the rest of the time.  You can choose
            # whether to show the initial "negative time convergence"....
            # if not t<0:
            pub.send(thetacur)
            servo.sleep()

        # if reset button is pushed go to startup and reset
        else:
            setup()


if __name__ == "__main__":
    setup()

