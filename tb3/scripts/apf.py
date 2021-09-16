#!/usr/bin/env python
# coding=utf-8
import math
from misc import saturation
#####################----APF参数设置-----#####################
"""
        :param start_posi_flag: 起点
        :param target_point: 终点
        :param obs: 障碍物列表，每个元素为Vector2d对象
        :param k_att: 引力系数
        :param k_rep: 斥力系数
        :param rr: 斥力作用范围
        :param goal_threashold: 离目标点小于此值即认为到达目标点
"""
class APF():
    def __init__(self):
        self.pos = [0,0,0]
        self.goal = [0,0]
        self.obstacles = [[]]
        self.k_att = 8
        self.k_rep = 8
        self.rr = 1.5  # 斥力作用范围
        self.max_speed = 0.5
        self.max_angular = 1
        self.near_obs = [0,0]

    def attractive(self):
        """
        引力计算
        :return: 引力
        """
        att = [0, 0]
        att = [(self.goal[0]-self.pos[0])* self.k_att, (self.goal[1]-self.pos[1])* self.k_att]
        return att

    def repulsion(self):
        """
        斥力计算, 改进斥力函数, 解决不可达问题
        :return: 斥力大小
        """
        rep = [0,0]
        min_dist = 999
        # print self.obstacles,self.pos
        for obstacle in self.obstacles:
            obs_to_rob = [obstacle[0]-self.pos[0],obstacle[1]-self.pos[1]]
            obs_to_rob_lengh = math.hypot(obs_to_rob[0],obs_to_rob[1])
            rob_to_goal = [self.goal[0]-self.pos[0],self.goal[1]-self.pos[1]]
            obs_to_goal_lengh = math.hypot(rob_to_goal[0], rob_to_goal[1])
            if (obs_to_rob_lengh > self.rr):  # 超出障碍物斥力影响范围
                pass
            else:
                if obs_to_rob_lengh < min_dist:
                    self.near_obs = obstacle
                    min_dist = obs_to_rob_lengh
                rep_1_p = self.k_rep * (1.0 / obs_to_rob_lengh - 1.0 / self.rr) / (obs_to_rob_lengh ** 2) * (obs_to_goal_lengh ** 2)
                rep_1 = [obs_to_rob[0]*rep_1_p, obs_to_rob[1]*rep_1_p]
                rep_2_p = self.k_rep * ((1.0 / obs_to_rob_lengh - 1.0 / self.rr) ** 2) * obs_to_goal_lengh
                rep_2 = [rob_to_goal[0]*rep_2_p, rob_to_goal[1]*rep_2_p]
                # 斥力的作用力是负值
                rep = [-(rep[0]+rep_1[0] + rep_2[0]),-(rep[1]+rep_1[1] + rep_2[1])]
        return rep

    def out_put_velocity(self):
        att = self.attractive()
        rep = self.repulsion()
        # 防止局部最小值
        if abs(att[1]-rep[1])<2:
            rep[1] = rep[1] + rep[0]
        if abs(att[0]-rep[0])<2:
            rep[0] = rep[0] + rep[1]
        f_vec = [att[0]+rep[0],att[1]+rep[1]]
        # print rep
        ######----角度计算-----##########
        # 注意这里必须是atan2，否则角度会出问题
        angular = math.atan2(f_vec[1], f_vec[0]) - self.pos[2]
        if angular > math.pi:
            angular = angular - 2 * math.pi
        if angular < -math.pi:
            angular = angular + 2 * math.pi
        # angle_obs_robot =  math.atan2(self.near_obs[1]-self.pos[1],self.near_obs[0]-self.pos[0])
        # goal_obs_robot = math.atan2(self.goal[1] - self.pos[1], self.goal[0] - self.pos[0])
        speed_x = math.sqrt((self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2)
        # speed_x = saturation(4*(goal_obs_robot-angle_obs_robot)/math.pi*self.max_speed,0.3,self.max_speed)
        return speed_x,angular



    def planning(self, pos, goal, obstacles):
        self.pos = pos
        self.goal = goal
        self.obstacles = obstacles
        speed_x,angular = self.out_put_velocity()

        speed_x =  speed_x
        angular_x =  angular
        # 速度和角速度限制
        if speed_x > self.max_speed:
            speed_x = self.max_speed
        if angular_x > self.max_angular:
            angular_x = self.max_angular
        if angular_x < -self.max_angular:
            angular_x = -self.max_angular
            # 令机器人转向的的时候线速度为0
        if abs(angular_x) > 60.0 / 180 * math.pi:
            speed_x = 0
        return speed_x,angular_x














