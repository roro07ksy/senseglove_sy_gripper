#!/usr/bin/env python3
from __future__ import print_function, division
import rospy
import actionlib
import dynamixel_sdk as dxl                  # Uses DYNAMIXEL SDK library
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16MultiArray
#from std_msgs.msg import Int64MultiArray
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
import time
import sys

# sys.stdout = unbuffered

# Control table address
ADDR_XL330_TORQUE_ENABLE       	= 64                          # Control table address is different in Dynamixel model

ADDR_XL330_PRESENT_VELOCITY	= 112	
ADDR_XL330_GOAL_POSITION       	= 116

ADDR_XL330_PRESENT_POSITION	= 132
ADDR_XL330_OPERATING_MODE	= 11
ADDR_XL330_CURRENT_LIMIT	= 38

ADDR_XL330_GOAL_CURRENT		= 102
ADDR_XL330_DRIVING_MODE     = 10

ADDR_XL330_D_GAIN_POSITION  = 80
ADDR_XL330_I_GAIN_POSITION  = 82
ADDR_XL330_P_GAIN_POSITION  = 84


LEN_GOAL_POSITION		= 4
LEN_PRESENT_VELOCITY		= 4
LEN_PRESENT_POSITION		= 4
LEN_GOAL_CURRENT        = 2
LEN_DRIVING_MODE        = 1

# Operating mode
CURRENT_CONTROL_MODE		= 0
POSITION_CONTROL_MODE		= 3
CURRENT_POSITION_CONTROL_MODE	= 5
EXTENDED_POSITION_CONTROL_MODE  = 4

# Protocol version
PROTOCOL_VERSION            = 2    


BAUDRATE                    = 57600
#DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
                                                        # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"


TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque

NUM_FINGER				= 4
NUM_JOINT				= 8

init_fe = [0,0,0,0]
init_aa = [2000, 2000, 2000, 2000]

pos = [0,0,0,0]
vel = [0,0,0,0]

desired_pos_fe = [0,0,0,0]
desired_pos_aa = [0,0,0,0]
        
#Preset dynamixel joint value of Gripper
#ps_fe = np.array([[0,2550,3500],[0,2900,4400],[0,2841,4400],[0,3167,4400]]) # plate : 0 , pinch , full flexion
#220907 update
ps_fe = np.array([[0,2300,4000],[0,2700,4500],[0,2900,4500],[0,3150,4500]]) # plate : 0 , pinch , full flexion
ps_aa = np.array([[330,0,-500],[70,0,0],[0,0,0],[-70,0,0]]) #AA same order with calibration posture 

CurLimit_FE = [850, 550, 550, 550]
Gain_position = [400, 40, 250]  # D - I - P


class HandInterface:
    def __init__(self):
        self.past_glove_joints = np.zeros(NUM_JOINT)
        self.past_glove_joints_AA = np.zeros(NUM_FINGER)
        self.past_glove_joints_FE = np.zeros(NUM_FINGER)
        self.current_dxl_joints = np.zeros(NUM_JOINT)
      
        #self.joint_currents = np.zeros(NUM_JOINT)
        self.joint_currents = np.zeros(NUM_JOINT,dtype=np.int16)

        self.calib_poses = {}
        self.calib_types = ['plate',
                            'pinch',
                            'finger_flexion',
                            'thumb_flexion',
                            'sphere']

        

        # print('init dxl')
        self.__init_dxl()
        # print('calib dxl')
        self.__calibration()

        self.heartbeat()
        
        
        self.tau = 0.6


        self.full_joint_names = ['thumb_brake', 'index_brake', 'middle_brake', 'ring_brake', 'pinky_brake', 
                                    'thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        self.vib_names = ['thumb_cmc', 'index_mcp', 'middle_mcp', 'ring_mcp', 'pinky_mcp']
        rospy.Subscriber(SENSEGLOVE_TOPIC, JointState, self.callback, queue_size=1)

        # haptic feedback from optoforce
        self.feedback_client = actionlib.SimpleActionClient(FEEDBACK_TOPIC, FollowJointTrajectoryAction)
        #self.feedback_client.wait_for_server()
        self.feedback_goal = FollowJointTrajectoryGoal()

        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration.from_sec(0.3)
        self.feedback_goal.trajectory.points.append(point)

        #self.set_glove_feedback(self.full_joint_names, [0] * 10)
        #rospy.Subscriber("/optoforce_1", WrenchStamped, self.callback1, queue_size=1)
        rospy.Subscriber(OPTOFORCE_TOPOC, Float64MultiArray, self.callback1, queue_size=1)


        #joint current publisher
        self.current_pub = rospy.Publisher(CURRENT_TOPIC, Int16MultiArray, queue_size = 1)



    def set_glove_feedback(self, names, vals):
        self.feedback_goal.trajectory.joint_names = names
        self.feedback_goal.trajectory.points[0].positions = vals
        self.feedback_goal.trajectory.header.stamp = rospy.Time.now()
        self.feedback_client.send_goal(self.feedback_goal)
        self.feedback_client.wait_for_result()

        # how to use 
        # self.set_glove_feedback(['thumb_cmc'], [40]) # vibration 40 %
        # self.set_glove_feedback(['thumb_brake', 'thumb_cmc'], [20, 40]) # break 20 %, vibration 40 %
    
    def heartbeat(self):
        # print('heart beat')
        #dxl_comm_result = self.groupSyncRead.txRxPacket()
        #print(self.groupSyncRead_current.last_errors)
        for i in DXL_ID:
            try:
                if self.groupSyncRead_current.last_errors[i] != 0:
                    print('[ID:%03d] !!!! HARDWARE ERROR !!!!!!' % i)
                    self.recovery(i)
            except:
                pass

    def recovery(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, id)
        rospy.sleep(0.1)
        self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        if dxl_comm_result != 0:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] reboot Succeeded\n" % id)
        print("[ID:%03d] torque on\n" % id)
        sys.stdout.flush()
    
    def __init_dxl(self):
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for i in DXL_ID	 :
            self.groupSyncRead.addParam(i)

        self.groupSyncRead_current = dxl.GroupSyncRead(self.portHandler, self.packetHandler, 126, 2)
        for i in DXL_ID_AA :
            self.groupSyncRead_current.addParam(i)

        for i in DXL_ID_FE :
            self.groupSyncRead_current.addParam(i)


	
        self.groupSyncWrite_AA = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncWrite_FE = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
		
		


        # Open port
        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass
        if self.portHandler.openPort(): print("Succeeded to open the port")
        else: print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE): print("Succeeded to change the baudrate")
        else: print("Failed to change the baudrate")

		# ALL joint Torque off
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
			
		# Change Operating mode
        for i in DXL_ID_AA :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , CURRENT_POSITION_CONTROL_MODE)
			
        for i in DXL_ID_FE :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , CURRENT_CONTROL_MODE)

 #       for i in range(4):
 #          self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID[i], ADDR_XL330_OPERATING_MODE , CURRENT_POSITION_CONTROL_MODE) 
 #          self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID[i], ADDR_XL330_CURRENT_LIMIT , 900) 
 

        # Set Current Limit
        for i in range(4) : 
            self.packetHandler.write2ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_CURRENT_LIMIT , CurLimit_FE[i])


        for i in DXL_ID_AA :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_CURRENT_LIMIT , 300)


        # AA joint Torque on and init pos
        for i in DXL_ID_AA:
                self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
                self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_POSITION , 2000)

        # FE joint Torque on and current init
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 0)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_CURRENT, 80)

        rospy.sleep(3.0)

        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE, 1)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)

        for i in range(4) :
            init_fe[i] = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_FE[i],ADDR_XL330_PRESENT_POSITION)[0]
		
        # FE joint Torque off and Change Operating Mode
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE , CURRENT_POSITION_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)


        # Set PID gains of Controller
        for i in DXL_ID_FE :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_D_GAIN_POSITION , Gain_position[0])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_I_GAIN_POSITION , Gain_position[1])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_P_GAIN_POSITION , Gain_position[2])
        for i in DXL_ID_AA :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_D_GAIN_POSITION , Gain_position[0])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_I_GAIN_POSITION , 0) # No I gain to AA joints
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_P_GAIN_POSITION , 500)
	
        dxl_comm_result = self.groupSyncRead_current.txRxPacket()
		
    def __calibration(self):
        
        
        for calib_type in self.calib_types:
            while rospy.has_param(CALIB_TOPIC + calib_type) != 1 :
                print('[',rospy.get_time(),']', 'Waiting Hand Calibrations',location)
                if (rospy.is_shutdown()):
                    break
                rospy.sleep(2)
            self.calib_poses[calib_type] = rospy.get_param(CALIB_TOPIC + calib_type)
            


        # TODO: 
        # Use self.calib_poses['stretch'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger2'], : numpy.array(), len() = 4
        #     self.calib_poses['thumb_finger3'], : numpy.array(), len() = 4
        #     self.calib_poses['lateral_pinch'] : numpy.array(), len() = 4

        # example ############
        self.pla_pos = self.calib_poses['plate']
        self.pin_pos = self.calib_poses['pinch']
        self.ffe_pos = self.calib_poses['finger_flexion']
        self.tfe_pos = self.calib_poses['thumb_flexion']
        self.sph_pos = self.calib_poses['sphere']

        self.aa_cal_pos = np.array([[self.pla_pos[0],0,self.sph_pos[0]],[self.pla_pos[1],0,self.sph_pos[1]],[self.pla_pos[2],0,self.sph_pos[2]],[self.pla_pos[3],0,self.sph_pos[3]]]) 
        self.aa_cal_pos = AA_DIRECTION * self.aa_cal_pos
        self.fe_cal_pos = np.array([[self.pla_pos[4], self.pin_pos[4],self.tfe_pos[4]], [self.pla_pos[5], self.pin_pos[5],self.ffe_pos[5]], [self.pla_pos[6], self.pin_pos[6],self.ffe_pos[6]], [self.pla_pos[7], self.pin_pos[7],self.ffe_pos[7]]]) 

    def callback(self, data):
        #self.read_joint_position()
        input_pose = data.position
        
        self.current_glove_joint = np.array([input_pose[16], input_pose[0], input_pose[4], input_pose[8], input_pose[18], input_pose[2], input_pose[6], input_pose[10]])
        self.current_glove_joint_AA = np.array([input_pose[16], input_pose[0], input_pose[4], input_pose[12]])
        self.current_glove_joint_AA = AA_DIRECTION * self.current_glove_joint_AA
        self.current_glove_joint_FE = np.array([input_pose[18], input_pose[2], input_pose[6], input_pose[14]])
        
        self.filtered_glove_joint = self.current_glove_joint * self.tau + self.past_glove_joints * (1 - self.tau)
        self.filtered_glove_joint_AA = self.current_glove_joint_AA * self.tau + self.past_glove_joints_AA * (1 - self.tau)
        self.filtered_glove_joint_FE = self.current_glove_joint_FE * self.tau + self.past_glove_joints_FE * (1 - self.tau)
        
        for i in range(4):
            desired_pos_aa[i] = init_aa[i] + int(((ps_aa[i,2]-ps_aa[i,0])/(self.aa_cal_pos[i,2] - self.aa_cal_pos[i,0]))*(self.filtered_glove_joint_AA[i] - self.aa_cal_pos[i,0]))


        desired_pos_aa[0] = init_aa[0] + ps_aa[0,0] + int(((ps_aa[0,2]-ps_aa[0,0])/(self.aa_cal_pos[0,2] - self.aa_cal_pos[0,0]))*(self.filtered_glove_joint_AA[0] - self.aa_cal_pos[0,0]))
        desired_pos_aa[2] = init_aa[2] #temp : middle finger fix


        for i in range(4):
            if i ==0 : #thumb flexion data decrease when thumb flexed
                if self.filtered_glove_joint_FE[i] > self.fe_cal_pos[i,1] :
                    desired_pos_fe[i] = init_fe[i] + int(((ps_fe[i,1]-ps_fe[i,0])/(self.fe_cal_pos[i,1] - self.fe_cal_pos[i,0]))*(self.filtered_glove_joint_FE[i] - self.fe_cal_pos[i,0]))
                else :
                    desired_pos_fe[i] = init_fe[i]+ ps_fe[i,1] + int(((ps_fe[i,2]-ps_fe[i,1])/(self.fe_cal_pos[i,2] - self.fe_cal_pos[i,1]))*(self.filtered_glove_joint_FE[i] - self.fe_cal_pos[i,1]))


            else : 
                if self.filtered_glove_joint_FE[i] < self.fe_cal_pos[i,1] :
                    desired_pos_fe[i] = init_fe[i] + int(((ps_fe[i,1]-ps_fe[i,0])/(self.fe_cal_pos[i,1] - self.fe_cal_pos[i,0]))*(self.filtered_glove_joint_FE[i] - self.fe_cal_pos[i,0]))
                else :
                    desired_pos_fe[i] = init_fe[i]+ ps_fe[i,1] + int(((ps_fe[i,2]-ps_fe[i,1])/(self.fe_cal_pos[i,2] - self.fe_cal_pos[i,1]))*(self.filtered_glove_joint_FE[i] - self.fe_cal_pos[i,1]))


        for i in range(4):
            if desired_pos_fe[i] < init_fe[i] :
                desired_pos_fe[i] = init_fe[i]
            elif desired_pos_fe[i] > (init_fe[i] +4400) :
                desired_pos_fe[i] = (init_fe[i] +4400)

        #print(desired_pos_aa)
        #print(desired_pos_fe)
        #print(init_fe)
        #print(self.filtered_glove_joint_FE)
        #print(self.fe_cal_pos)
        

	#print(desired_pos_aa)
	

        self.groupSyncWrite.clearParam()
        for i in range(4):
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_pos_aa[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_pos_aa[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_pos_aa[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_pos_aa[i]))]
            self.groupSyncWrite.addParam(DXL_ID_AA[i], param_goal_position)
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_pos_fe[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_pos_fe[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_pos_fe[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_pos_fe[i]))]
            self.groupSyncWrite.addParam(DXL_ID_FE[i], param_goal_position)


        dxl_comm_result = self.groupSyncWrite.txPacket()
        self.past_glove_joints = self.filtered_glove_joint
        self.past_glove_joints_AA = self.filtered_glove_joint_AA
        self.past_glove_joints_FE = self.filtered_glove_joint_FE


    def callback1(self, data):
        vib_data = [0, 0, 0, 0]
        break_data = [0, 0, 0, 0]
        #print(data.data)
        for i in range(4):
            if data.data[i]*30 > 5 :
                #vib_data[i] = data.data[i]*30+35
                vib_data[i] = 40
                break_data[i] =100

            else :
                vib_data[i] = 0
                break_data[i] =0
            

#        force_array = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
#        force_mag = np.linalg.norm(force_array,2)*30
#       if force_mag > 5:
#            vib_data = [0,np.linalg.norm(force_array,2)*30+35,0]
#            break_data = [0, 80,0]
#        else :
#            vib_data = [0,0,0]
#            break_data = [0, 0,0]
#        print(vib_data)
#        for d in sensor_data:
#           vib_data.append( min (d * 40, 100))
#           if vib_data[-1] < 50.0:
#               vib_data[-1] = 0
        
        # print('vib_data :' , vib_data)
        #self.set_glove_feedback(self.full_joint_names[0:3] + self.vib_names[0:3], [0,0,0] + vib_data[0:3])
        self.set_glove_feedback(self.full_joint_names[0:4] + self.vib_names[0:4], break_data[0:4] + vib_data[0:4])



    def read_joint_position(self) :
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        for i in range(8) :
            self.current_dxl_joints[i] = self.groupSyncRead.getData(DXL_ID[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            #print('Joint pos read')
        
        # print('current pos ' , pos)


    def read_current(self) :
        dxl_current_result = self.groupSyncRead_current.txRxPacket()
        for i in range(4) :
            self.joint_currents[i] = self.groupSyncRead_current.getData(DXL_ID_AA[i], 126, 2)
        for i in range(4) :
            self.joint_currents[i+4] = self.groupSyncRead_current.getData(DXL_ID_FE[i], 126, 2)
        msg= Int16MultiArray()
        #msg= Int64MultiArray()
        msg.data = self.joint_currents.tolist()
        self.current_pub.publish(msg)
        #print(msg.data)

        
if __name__== '__main__':
    rospy.init_node('run_hand')


    location = rospy.get_param('~location', 'unset')
    dev = rospy.get_param('~dev', 'unset')

    if location != 'left' and location != 'right':
            raise NameError('location is wrong! location: {0}'.format(location))

    if location == 'right':
        DXL_ID = [11,12,21,22,31,32,41,42]
        DXL_ID_FE = [12,22,32,42]
        DXL_ID_AA = [11,21,31,41]
        
        #DEVICENAME  = "/dev/ttyUSB0".encode('utf-8')        # Check which port is being used on your controller
        DEVICENAME  = dev
        SENSEGLOVE_TOPIC = "/senseglove/0/rh/joint_states"
        #SENSEGLOVE_TOPIC = "/mix_data"
        FEEDBACK_TOPIC = '/senseglove/0/rh/controller/trajectory/follow_joint_trajectory'
        OPTOFORCE_TOPOC = "/optoforce_norm_rh"
        CALIB_TOPIC = '/dyros_glove/calibration/right/'
        CURRENT_TOPIC = '/hand/r/current'

        AA_DIRECTION = 1

        


    elif location == 'left':
        DXL_ID = [51,52,61,62,71,72,81,82]
        DXL_ID_FE = [52,62,72,82]
        DXL_ID_AA = [51,61,71,81]
        
        #DEVICENAME  = "/dev/ttyUSB1".encode('utf-8')         # Check which port is being used on your controller
        DEVICENAME  = dev
        SENSEGLOVE_TOPIC = "/senseglove/0/lh/joint_states"
        FEEDBACK_TOPIC = '/senseglove/0/lh/controller/trajectory/follow_joint_trajectory'
        OPTOFORCE_TOPOC = "/optoforce_norm_lh"
        CALIB_TOPIC = '/dyros_glove/calibration/left/'
        CURRENT_TOPIC = '/hand/l/current'

        AA_DIRECTION = -1

    else: raise NameError('??{0}'.format(location))


    hi = HandInterface()

    sys.stdout.flush()

    flush_60 = 0;

    while rospy.is_shutdown() is False:
        #print('read current')
        hi.read_current()
        hi.heartbeat()
        time.sleep(0.02)
        flush_60 = flush_60 + 1
        if(flush_60 == 60):
            flush_60 = 0
            sys.stdout.flush()
    #while rospy.is_shutdown() is False:
    #   hi.read_current()
    #   hi.heartbeat()
    #    time.sleep(1.0)

    # rospy.spin()
    
    # shutdown
    for i in DXL_ID:
        hi.packetHandler.write1ByteTxRx(hi.portHandler, DXL_ID, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
