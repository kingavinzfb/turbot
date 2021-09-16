#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from turtlebot import Leader_x, Follower,Set_Goal
from misc import angle_adjust, dist, saturation, limit
import math
from dwa import DWA
from apf import APF
import time
import numpy as np
max_speed = 0.5 
max_angular = 2
if __name__ == '__main__':
    rospy.init_node('simulation')
	# 整个代码的执行频率
    dt = 0.1
	#四个机器人的初始位置
    init_pos = [[3, 5, 0], [0, -2, 3.14], [0, 2, 0],[0, -3, 1.57]]
    goal_x = [[4, 4], [6, 0], [4, -3], [0, 0]]
    obstacles = [[2, 4], [6, 4], [6, -3], [2, -3], [4, 0]]
    dwa_controller = DWA(dt = dt)
    apf_controller = APF()
    # 初始化目标点和机器人
    leader_x = Leader_x()
    leader_x.init_gazebo_turtlebot("/burger1/odom", "/burger1/cmd_vel", "/burger1/scan")
    leader_x.teleportfixed("burger1", init_pos[0])
    set_goal = Set_Goal()
    # leader_y.teleportfixed("burger2", init_pos[1])s
    follower = [None for i in range(3)]
    for id in range(3):
        follower[id] = Follower()
        follower[id].init_gazebo_turtlebot("burger" + str(id + 2) + "/odom",
                                           "burger" + str(id + 2) + "/cmd_vel","burger" + str(id + 2) + "/scan")
        if id == 0:
            follower[id].teleportfixed("burger2", init_pos[1])
        if id == 1:
            follower[id].teleportfixed("burger3", init_pos[2])
        if id == 2:
            follower[id].teleportfixed("burger4", init_pos[3])
    for i in range(100):
        follower[0].stop()
        follower[1].stop()
        follower[2].stop()
        leader_x.stop()
    dist_thod = 0.05
    dist_thod_f = 0.5
    angle_thod = 0.05
    pos = [[0, 0, 0] for i in range(4)]
    k = 0   
    current_time = 0
    last_time = 0
    # 初始清空model
    set_goal.clearModel()
    name = ["sign1","sign2","sign3","sign4"]
    set_goal.respawnModel(4,4,name[0])
    set_goal.respawnModel(6,0,name[1])
    set_goal.respawnModel(4,-3,name[2])
    set_goal.respawnModel(0,0,name[3])
    while not rospy.is_shutdown() :
        current_time = time.time()
# 判断机器人是否订阅到位置
        while (not leader_x.checkState() or not follower[0].checkState() or not follower[1].checkState() or not follower[2].checkState()):
            if rospy.is_shutdown():
                break
        pos[0] = leader_x.get_poistion()
        pos[1] = follower[0].get_poistion()
        pos[2] = follower[1].get_poistion()
        pos[3] = follower[2].get_poistion()
        # 随机移动的机器人

        if  dist(goal_x[k], pos[0]) > dist_thod:
            # 生成model
            #set_goal.respawnModel(goal_x[k][0],goal_x[k][1])
            goal_angle_x = math.atan2(
                goal_x[k][1] - pos[0][1], goal_x[k][0] -
                pos[0][0]) - pos[0][2]
            if abs(goal_angle_x)<math.pi*30/180:
                speed_x = saturation(0.5 * dist(pos[0], goal_x[k]), 0.3, max_speed)
            else:
                speed_x = 0
            angular_x = angle_adjust(goal_angle_x)
            leader_x.publishVelocityCommand(speed_x, angular_x)
        elif dist(goal_x[k], pos[0]) < dist_thod:
            # and abs(angle_adjust(goal_x[k][2] - pos[0][2])) < angle_thod:
	    name = ["sign1","sign2","sign3","sign4"]            
	    if k < len(goal_x)-1:
                # 删除model
                set_goal.deleteModel(name[k])
                k += 1
            else:  
                set_goal.deleteModel(name[k])
                leader_x.stop()

        # follow 1 DWA controller
        if dist(pos[2], pos[0]) > dist_thod_f:
            speed_x, angular_x = dwa_controller.planning(pos[2],pos[0],obstacles = obstacles)
            follower[1].publishVelocityCommand(speed_x, angular_x)
	    print(dist(pos[2], pos[0]))
	    print(time.localtime(time.time()))
        else:
            follower[1].stop()


        # delay function
        if time.time() - current_time < dt:
            time.sleep(dt - (time.time()-current_time))
        last_time = current_time
    print("task stop")
