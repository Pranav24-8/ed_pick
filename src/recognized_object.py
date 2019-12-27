#! /usr/bin/env python
class RecognizedObject():

    def __init__(self, name, id = 0, dimensions = (0.07, 0.07, 0.25), rotational_symmetric = True, power_grasp = True, priority = 0.0):
        """
        param name: object name
        param dimensions: (object_length, object_width, object_height)
        """
        self.name = name
        self.pose = None  #pose (center of object)

        '''dimensions tupel (x, y, z) given in the gripper frame.
        When the object is grasped by the gripper the x axis shows in grasp direction, z shows up'''
        self.dimensions = dimensions
        self.id = id

        ''''when the object is rotational symmetric,
        the grasp generator generates grasps rotated around the symmetry axis of the object'''
        self.rotational_symmetric = rotational_symmetric


        self.priority = priority

        ''''when the grasp is a power grasp the fingertips won't exceed the object,
        so when the object lies flat on the table the fingertips don't touch the table
        this should be disabled for small objects to do a pinch grasp'''
        self.power_grasp = power_grasp

        self.distance_to_origin = 0.0

        self.top_grasp = False


    def __repr__(self):
        return "\nname: {name}, \npose: {pose}\ndistance to arm base link: {distance_to_origin}\n".format(name = self.name, pose = self.pose, distance_to_origin = self.distance_to_origin)


    def __eq__(self, other):
        return self.name == other.name
