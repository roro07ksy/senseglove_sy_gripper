#!/usr/bin/env python
from __future__ import print_function, division
import rospy
import os, sys
import copy
import numpy as np
#import keyboard
from functools import partial
from sensor_msgs.msg import JointState

class GloveCalibration:
    def __init__(self):
        num_joints = 8
        self.past_glove_joints = {'left':np.zeros(num_joints), 'right':np.zeros(num_joints)}
        self.current_dxl_joints = {'left':np.zeros(num_joints), 'right':np.zeros(num_joints)}

        self.joint_captures = {'left':list(), 'right':list()}
        self.capture_trigger = {'left':False, 'right':False}
        self.current_glove_joint = {'left':np.zeros(num_joints),'right':np.zeros(num_joints)}
        self.filtered_glove_joint = {'left':np.zeros(num_joints),'right':np.zeros(num_joints)}
        self.num_captures = 20
        self.tau = 0.6

        self.calib_types = ['plate',
                            'pinch',
                            'finger_flexion',
                            'thumb_flexion',
                            'sphere']

        rospy.Subscriber("/senseglove/0/lh/joint_states", JointState, partial(self.joint_callback,'left'), queue_size=1)
        rospy.Subscriber("/senseglove/0/rh/joint_states", JointState, partial(self.joint_callback,'right'), queue_size=1)

    def joint_callback(self, location, data):
        input_pose = data.position
        self.current_glove_joint[location] = np.array([input_pose[16], input_pose[0], input_pose[4], input_pose[12], input_pose[18], input_pose[2], input_pose[6], input_pose[14]])  #AA next FE
        self.filtered_glove_joint[location] = self.current_glove_joint[location] * self.tau + self.past_glove_joints[location] * (1 - self.tau)
        self.past_glove_joints[location] = self.filtered_glove_joint[location]

        if self.capture_trigger[location]:
            self.joint_captures[location].append(self.filtered_glove_joint[location])
            
            print ('captured {0} samples'.format(len(self.joint_captures[location])))

            if len(self.joint_captures[location]) >= self.num_captures:
                self.capture_trigger[location] = False

    def calibration(self, location):
        print('-------- Calibration starts [{0}] --------'.format(location))
        for calib_type in self.calib_types:
            print ('calibrating... {0} - {1}'.format(location, calib_type))
            print('Press enter to start this calibration')
            a = input()
            #os.system('pause')

            self.joint_captures[location] = []
            self.capture_trigger[location] = True

            print('capturing {0} samples! don\'t move'.format(self.num_captures))
            
            # wait for capturing
            while rospy.is_shutdown() is False:
                if self.capture_trigger[location] is False:
                    break
            
            sum_joints = np.zeros(8)    
            for joints in self.joint_captures[location]:
                sum_joints += joints

            mean_joints = sum_joints / float(self.num_captures)
            print(mean_joints)

            rospy.set_param('/dyros_glove/calibration/{0}/{1}'.format(location,calib_type), mean_joints.tolist())

        print('-------- Calibration done [{0}]--------'.format(location))


if __name__== '__main__':
    rospy.init_node('glove_hand_calibration')
    gc = GloveCalibration()
    gc.calibration('left')
    gc.calibration('right')
