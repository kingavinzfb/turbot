#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, Twist,TwistStamped
from nav_msgs.msg import Odometry
import math
import misc
import time
import numpy as np
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import LaserScan

class Set_Goal():
    """
    机器人目标控制
    """
    def __init__(self):
        self.model_path = "./models/gazebo/goal_sign/model.sdf"
        f = open(self.model_path, 'r')
        self.model = f.read()
        self.goal_position = Pose()
        self.goal_position.position.x = None  # Initial positions
        self.goal_position.position.y = None
        self.model_name = 'goal_sign'
        self.check_model = 0  # 检查是否有模型

    def respawnModel(self,x,y,name):
        '''
        在gazebo中生成地面上的圈
        '''
        self.goal_position.position.x, self.goal_position.position.y = [x, y]
        if 1==1:
            try:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model',
                                                      SpawnModel)
		self.model_name = name
                spawn_model_prox(self.model_name, self.model,
                                 'robotos_name_space', self.goal_position,
                                 "world")
                self.check_model = self.check_model+1
            except Exception as e:
                rospy.logfatal("生成目标点失败 " + str(e))

    def deleteModel(self,name):
        '''
        在gazebo中删除圈
        '''
        if self.check_model!=0:
            try:  
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model',
                                                    DeleteModel)
		self.model_name = name
                del_model_prox(self.model_name)
                self.check_model=self.check_model-1
            except Exception as e:
                rospy.logfatal("Error when deleting the goal sign " + str(e))

    def clearModel(self):
        '''
        在gazebo中删除圈
        '''
        try:
            rospy.wait_for_service('gazebo/delete_model')
            del_model_prox = rospy.ServiceProxy('gazebo/delete_model',
                                                DeleteModel)
            del_model_prox(self.model_name)
            self.check_model=self.check_model-1
        except Exception as e:
            rospy.logfatal("Error when deleting the goal sign " + str(e))



class turtlebot:
    def __init__(self):
        # turtlebot_config
        self.kw = 0.45
        self.delta_theta_threshold = 10 * math.pi / 180
        self.safe_dist = 0.4
        self.stop_dist = 0.03
        self.T = 20
        self.max_linear_vel = 0.2
        self.max_angular_vel = 2.84

        self.goal_point = [0, 0]
        self.pose_ready_ = False
        self.vel_ready_ = False
        self.pub_cmd = None
        self.robot_info = {
            'pos': Point(),
            'angle': 0,
            'g2r': 0,
            'g2r_dist': 0,
            'v': 0,
            'omega': 0
        }

    def init_gazebo_turtlebot(self, gazebo_odom_topic, cmd_topic,scan_topic):
        self.pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(gazebo_odom_topic, Odometry,
                         self.gazebo_odom_callback)
        rospy.Subscriber(scan_topic,LaserScan ,
                         self.gazebo_scan_callback)
        self.pose_ready_ = False
        self.vel_ready_ = False

    def gazebo_scan_callback(self, scan):

        for i in range(len(scan.ranges)):
            pass

    def gazebo_odom_callback(self, data):
        self.pose_ready_ = True
        self.vel_ready_ = True
        pos = data.pose.pose.position
        self.robot_info['pos'] = pos
        orien = data.pose.pose.orientation
        x, y, z, w = orien.x, orien.y, orien.z, orien.w
        # 计算机器人的朝向
        self.robot_info['angle'] = math.atan2(2 * (w * z + x * y),
                                              1 - 2 * (z * z + y * y))
        # 计算机器人和目标线的夹角,注意这里必须用atan2
        self.robot_info['g2r'] = math.atan2(
            (self.goal_point[1] - pos.y),
            (self.goal_point[0] - pos.x)) - self.robot_info['angle']
        if self.robot_info['g2r'] > math.pi:
            self.robot_info['g2r'] = self.robot_info['g2r'] - 2 * math.pi
        if self.robot_info['g2r'] < -math.pi:
            self.robot_info['g2r'] = self.robot_info['g2r'] + 2 * math.pi
        self.robot_info['g2r_dist'] = math.sqrt(
            (self.goal_point[0] - pos.x) ** 2 + (self.goal_point[1] - pos.y) ** 2)
        self.robot_info['v'] = data.twist.twist.linear.x
        self.robot_info['omega'] = data.twist.twist.angular.z

    def publishVelocityCommand(self, v, omega):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)

    def stop(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.pub_cmd.publish(cmd)


    def checkState(self):
        return self.pose_ready_ and self.vel_ready_

    def get_poistion(self):
        pos = [
            self.robot_info["pos"].x, self.robot_info["pos"].y,
            self.robot_info["angle"]
        ]
        return pos

    def ToQuaternion(self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        return q

    def teleportfixed(self, agent_model_name, pos):
        '''
        传送机器人位置
        返回[posX,posY]
        '''
        model_state_msg = ModelState()
        model_state_msg.model_name = agent_model_name

        # 选择机器人的位置
        pose = Pose()
        pose.position.x, pose.position.y = pos[0:2]
        pose.orientation = self.ToQuaternion(pos[2], 0, 0)
        model_state_msg.pose = pose
        model_state_msg.twist = Twist()

        model_state_msg.reference_frame = "world"

        # 开始机器人传送
        isTeleportSuccess = False

        if not isTeleportSuccess:
            try:
                rospy.wait_for_service('/gazebo/set_model_state')
                telep_model_prox = rospy.ServiceProxy(
                    '/gazebo/set_model_state', SetModelState)
                telep_model_prox(model_state_msg)
                isTeleportSuccess = True
                print("set " + agent_model_name + " position success!")
            except Exception as e:
                rospy.logfatal("set robot position false! " + str(e))

        return pose.position.x, pose.position.y


class Leader_x(turtlebot):
    def __init__(self):
        turtlebot.__init__(self)

    def set_goal(self, p):
        self.goal_point = p

    def publishVelocityCommand(self, v, omega):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.pub_cmd.publish(cmd)



class Follower(turtlebot):
    def __init__(self):
        turtlebot.__init__(self)
    def set_goal(self, p):
        self.goal_point = p

