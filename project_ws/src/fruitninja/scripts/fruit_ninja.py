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
    return np.array([[x], [y], [z]])


def dot(a, b):
    return a.T @ b


def cross(a, b):
    return np.cross(a, b, axis=0)


# Calculates start and ending position of the slash
# Based on position of should to try and emulate real sword slash
def slash():
    # want the slash to arc so take spherical coordinates of start and end

    # center point
    p = point.position()
    print(p)
    rp = np.linalg.norm(p)  # raidus to point
    print(rp)
    thp = math.atan2(p[1] , p[0])
    phip = np.sqrt(p[0] ** 2 + p[1] ** 2) / (p[2])

    # start of slash
    rp1 = rp
    th1 = thp + np.pi / 6
    phi1 = phip - np.pi / 4

    # end of slash
    rp2 = rp
    th2 = thp - np.pi / 6
    phi2 = phip + np.pi / 4

    # convert to cartesian
    p1 = [rp1 * np.sin(th1) * np.cos(phi1), rp1 * np.sin(th1) * np.sin(phi1), rp1 * np.sin(phi1)]  # absolute pos
    p2 = [rp2 * np.sin(th2) * np.cos(phi2), rp2 * np.sin(th2) * np.sin(phi2), rp2 * np.sin(phi2)]
    


    #
    sph1 = [rp1, th1, phi1]
    sph2 = [rp2, th2, phi2]

    return p1, p2, sph1, sph2


# sine trajectory position and velocity
def sinTrajP(p0, pf, t, t0, tf):  # uses numpy.add and subtract so can also be used for any matrix including rotation
    pt = np.add(p0, np.subtract(pf, p0) * (
                (t - t0) - (tf - t0) / (2 * np.pi) * np.sin(2 * np.pi * (t - t0) / (tf - t0))) / (tf - t0))
    return pt


def sinTrajV(p0, pf, t, t0, tf):  # velocity from a sine trajectory
    vt = np.subtract(pf, p0) * (1 - np.cos(2 * np.pi * (t - t0) / (tf - t0))) / (tf - t0)
    return vt


# rotate negative x axis on tip towards fruit to slice
def fruitPoint(tippos, tiprot):
    pfruit = point.position()
    veccp = np.subtract(pfruit, tippos)/np.linalg.norm(np.subtract(pfruit, tippos))  # vector from blade center to fruit position
    negx = tiprot @ vec(-1, 0, 0)
    axis = cross(negx, veccp)  # axis to rotate -x axis of tip towards the fruit
    cosvec = dot(negx, veccp)  # cosine of angle between two vectors
    skewsym = np.array([[0, -axis[2][0], axis[1][0]], [axis[2][0], 0, -axis[0][0]], [-axis[1][0], axis[0][0], 0]])  # skew symmetric matrix of axis
    desrot = np.add(np.identity(3), np.add(skewsym, skewsym @ skewsym * 1 / (1 + cosvec)))
    return desrot


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


#
#  Calculate the Desired
#
#  This computes the desired position and orientation, as well as the
#  desired translational and angular velocities for a given time.
#
def desired(t, tippos, tiprot, rcur, p0):
    # The point is simply taken from the subscriber.
    p1, p2, sph1, sph2 = slash() # sph1, sph2 used later to make sword swing more realistic
    print(p1)
    print(p2)
    time.sleep(10)

    # move blade to top of swing
    # continually calculate matrix to point towards fruit so at top blade is pointing towards fruit
    if t < 9:
        pd = sinTrajP(p0, p1, t, 0, 9)
        vd = sinTrajV(p0, p1, t, 0, 9)

        # Calculate the matrix to point towards the fruit and over time move to point towards the fruit
        Rf = fruitPoint(tippos, tiprot)
        Rd = sinTrajP(rcur, Rf, t, 0, 9)
        wd = np.zeros((3,1))

        # perform slash with rotation held at direction to point towards fruit at top of swing
        #TODO make more realistic swordwsing
    elif 9 <= t <= 11:
        if abs(t - 1) < .0001:
            rcur = fruitPoint(tippos, tiprot)
        pd = sinTrajP(p1, p2, t, 9, 11)
        vd = sinTrajV(p1, p2, t, 9, 11)
        Rd = rcur
        wd = np.zeros((3, 1))

        #return to starting position
    elif 11 < t <= 20:
        pd = sinTrajP(p2, p0, t, 11, 20)
        vd = sinTrajV(p2, p0, t, 11, 20)
        Rd = sinTrajP(rcur, np.identity(3), t, 11, 20)
        wd = np.zeros((3,1))
    else:
        pd = p0
        vd = np.zeros((3,1))
        Rd = np.identity(3)
        wd = np.zeros((3,1))
        
        

    # Return the data.
    return (pd, Rd, vd, wd)


#
#  Main Code
#
if __name__ == "__main__":
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

    # Make sure the URDF and publisher agree in dimensions.
    if not pub.dofs() == kin.dofs():
        rospy.logerr("FIX Publisher to agree with URDF!")

    # Set up the point subscriber.
    point = PointSubscriber()
    started = StartSubscriber()
    rospy.loginfo("Waiting for a point...")
    while not rospy.is_shutdown() and not point.valid():
        pass
    rospy.loginfo("Got a point.")

    # Set the numpy printing options (as not to be overly confusing).
    # This is entirely to make it look pretty (in my opinion).
    np.set_printoptions(suppress=True, precision=6)

    # Our robot moves from all states at 0 to the slash position, so this will have good initial guess
    theta = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
    pub.send(theta)

    # For the initial desired, head to the starting position (t=0).
    # Clear the velocities, just to be sure.
    rcur = np.identity(3)

    (pstart, R) = kin.fkin(theta)

    #
    #  TIME LOOP
    #
    # I play one "trick": I start at (t=-1) and use the first second
    # to allow the poor initial guess to move to the starting point.
    #
    p = pd = pstart
    R = Rd = rcur
    t = 0
    tf = 20
    lam = 0.1 / dt
    while not rospy.is_shutdown():
        if started.state:
            # Using the result theta(i-1) of the last cycle (i-1):
            # Compute the forward kinematics and the Jacobian.
            (p, R) = kin.fkin(theta)
            J = kin.Jac(theta)

            # Use that data to compute the error (left after last cycle).
            e = etip(p, pd, R, Rd)

            # Advance to the new time step.
            t += dt

            # Compute the new desired.
            (pd, Rd, vd, wd) = desired(t, p, R, rcur, pstart)

            # Build the reference velocity.
            vr = np.vstack((vd, wd)) + lam * e

            # Compute the Jacbian inverse (pseudo inverse)
            # Jpinv = np.linalg.pinv(J)
            Jinv = np.linalg.pinv(J)

            # Update the joint angles.
            thetadot = Jinv @ vr
            theta += dt * thetadot

            # Publish and sleep for the rest of the time.  You can choose
            # whether to show the initial "negative time convergence"....
            # if not t<0:
            pub.send(theta)
            servo.sleep()

            # Break the loop when we are finished.
            # if (t >= tf):
            #     break
        else:
            t = 0

