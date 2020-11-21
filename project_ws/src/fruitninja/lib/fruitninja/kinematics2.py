#!/usr/bin/env python3
#
#   Kinematics Class - Version 2
#
#   Use the KDL (Kinematics/Dynamics Library) to compute the forward
#   kinematics and Jacobian from a URDF.
#
#   The uses numpy to format the return variables into 2D matrices.
#
import numpy as np

import kdl_parser_py.urdf as kdlp
import PyKDL              as kdl


#
#  Kinematics Class Definition
#
class Kinematics:
    def __init__(self, urdf, world = 'world', tip = 'tip'):
        # Try loading the URDF data into a KDL tree.
        (ok, self.tree) = kdlp.treeFromString(urdf)
        assert ok, "Unable to parse the URDF"

        # Save the base and tip frame names.
        self.baseframe = world
        self.tipframe  = tip

        # Create the isolated chain from world to tip.
        self.chain = self.tree.getChain(world, tip)

        # Extract the number of joints.
        self.N = self.chain.getNrOfJoints()
        assert self.N>0, "Found no joints in the chain"

        # Create storage for the joint position, tip position, and
        # Jacobian matrices (for the KDL library).
        self.qkdl = kdl.JntArray(self.N)
        self.Tkdl = kdl.Frame()
        self.Jkdl = kdl.Jacobian(self.N)

        # Also pre-allocate the memory for the numpy return variables.
        # The content will be overwritten so the initial values are
        # irrelevant.
        self.p = np.zeros((3,1))
        self.R = np.identity(3)
        self.J = np.zeros((6,self.N))

        # Instantiate the solvers for tip position and Jacobian.
        self.fkinsolver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jacsolver  = kdl.ChainJntToJacSolver(self.chain)

    def report(self, printfunc):
        # Report on the chain generally.
        printfunc("Chain from '%s' to '%s':"
                  % (self.baseframe, self.tipframe))
        printfunc("Number of Chain Joints %d"
                  % (self.chain.getNrOfJoints()))
        printfunc("Number of Chain Segments %d"
                  % (self.chain.getNrOfSegments()))

        # Report on each segment.
        for i in range(self.chain.getNrOfSegments()):
            segment = self.chain.getSegment(i)
            # print(segment)
            printfunc("Seg %d Name %s"
                      % (i, segment.getName()))
            printfunc("Seg %d Jnt-Name %s"
                      % (i, segment.getJoint().getName()))

    def dofs(self):
        # Return the number of joints.
        return self.N

    def jointnames(self):
        # Return the joint names
        names = []
        for i in range(self.chain.getNrOfSegments()):
            names.append(self.chain.getSegment(i).getJoint().getName())
        return(names)

    def fkin(self, q):
        # Check the size of the given joint positions.
        assert len(q)==self.N, "Incorrectly sized joint angle vector"

        # Load the KDL's joint position.
        for i in range(self.N):
            self.qkdl[i] = q[i]

        # Run the fkin solver.
        bad = self.fkinsolver.JntToCart(self.qkdl, self.Tkdl)
        assert not bad, "Forward Kinematics Solver failed"

        # Extract the position.
        for i in range(3):
            self.p[i] = self.Tkdl.p[i]

        # Extract the rotation.
        for i in range(3):
            for j in range(3):
                self.R[i,j] = self.Tkdl.M[i,j]

        # Return the position and orientation
        return (self.p,self.R)

    def Jac(self, q):
        # Check the size of the given joint positions.
        assert len(q)==self.N, "Incorrectly sized joint angle vector"

        # Load the KDL's joint position.
        for i in range(self.N):
            self.qkdl[i] = q[i]

        # Run the fkin solver.
        bad = self.jacsolver.JntToJac(self.qkdl, self.Jkdl)
        assert not bad, "Jacobian Solver failed"

        # Extract the Jacobian matrix.
        for i in range(6):
            for j in range(self.N):
                self.J[i,j] = self.Jkdl[i,j]

        # Return the Jacobian matrix.
        return (self.J)
