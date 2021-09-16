import  os, math, time, random, copy


class DWA:
    """ Collision avoidance algorithm """

    def __init__(self, dt = 0.1,init_pose=(0, 0, 0), config=(0, 0, 0 ,0.5)):
        # parameters of robot
        self.ROBOT_RADIUS = 0.10
        # Linear velocity limits
        self.MAX_VEL_LINEAR = 0.5  # ms^(-1) max speed of each wheel
        self.MAX_ACC_LINEAR = 0.5  # ms^(-2) max rate we can change speed of each wheel
        # Angular velocity limits
        self.MAX_VEL_ANGULAR = 1.5
        self.MAX_ACC_ANGULAR = 10
        # Current linear velocity and angular velocity
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        # Current positions
        self.x, self.y, self.theta = init_pose
        # Parameters for prediction trajectory
        self.dt = dt
        self.k, self.u0, self.v0, self.OBSTACLE_RADIUS = config
        self.STEPS_AHEAD_TO_PLAN = 10
        self.TAU = self.dt * self.STEPS_AHEAD_TO_PLAN
        # Safe distance between robot and closest obstacle after minus robot's radius and obstacle's radius
        self.SAFE_DIST = 0.2
        # Weights for predicted trajectory evaluation
        self.FORWARDWEIGHT = 12
        self.OBSTACLEWEIGHT = 6666

    def predict_position(self, vLpossible, vRpossible, delta_time):
        """ Predict robot's position in delta_time

            @param vLpossible:    possible linear velocity

            @param vRpossible:    possible angular velocity

            @return:    (new_x, new_y, new_theta, path)
        """
        # Go straight line
        if round(vRpossible, 3) == 0:
            new_x = self.x + vLpossible * delta_time * math.cos(self.theta)
            new_y = self.y + vLpossible * delta_time * math.sin(self.theta)
            new_theta = self.theta
        # Pure rotation motion
        elif round(vLpossible, 3) == 0:
            new_x = self.x
            new_y = self.y
            new_theta = self.theta + vRpossible * delta_time
        else:
            # Rotation and arc angle of general circular motion
            R = vLpossible / vRpossible
            delta_theta = vRpossible * delta_time
            new_x = self.x + R * (math.sin(delta_theta + self.theta) - math.sin(self.theta))
            new_y = self.y - R * (math.cos(delta_theta + self.theta) - math.cos(self.theta))
            new_theta = self.theta + delta_theta

        return (new_x, new_y, new_theta)

    def calculateClosestObstacleDistance(self, predict_x, predict_y, obstacles):
        """ Calculate  distance to closest obstacle

            @param predict_x: predicted x coordiante of robot

            @param predict_y: predicted y coordiante of robot

            @param obstacles: contains obstacles' information,that is [pos_x, pos_y, vx, vy]

            @return: distance between robot and closest obstacle
        """
        closestdist = 100000.0
        for (idx, obstacle) in enumerate(obstacles):
            dx = obstacle[0] - predict_x
            dy = obstacle[1] - predict_y
            d = math.sqrt(dx ** 2 + dy ** 2)
            # Distance between closest touching point of circular robot and circular obstacle
            dist = d - self.ROBOT_RADIUS - self.OBSTACLE_RADIUS
            if dist < closestdist:
                closestdist = dist

        return closestdist

    def planning(self, init_pose, goal, obstacles):
        """ Planning trajectory and select linear and angular velocities for robot

            @param goal:  goal postion of robot

            @param obstacles:  [pos_x, pos_y, vx, vy] of each obstacles

            @return:  predicted_path_to_draw
        """
        self.x, self.y, self.theta = init_pose
        bestBenefit = -100000
        # Range of possible motions: each of vL and vR could go up or down a bit
        sample_num = 10
        vLUpBound = self.linear_vel + self.MAX_ACC_LINEAR * self.dt
        vLDownBound = self.linear_vel - self.MAX_ACC_LINEAR * self.dt
        vLpossiblearray = tuple(
            vLDownBound + 1.0*i / (sample_num - 1) * (vLUpBound - vLDownBound) for i in range(sample_num))
        # print('vLpossiblearray:', tuple(vLpossiblearray))
        vRUpBound = self.angular_vel + self.MAX_ACC_ANGULAR * self.dt
        vRDownBound = self.angular_vel - self.MAX_ACC_ANGULAR * self.dt
        vRpossiblearray = tuple(
            vRDownBound + 1.0*i / (sample_num - 1) * (vRUpBound - vRDownBound) for i in range(sample_num))
        # print('vRpossiblearray:', tuple(vRpossiblearray))
        vLchosen = 0
        vRchosen = 0
        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # Check if in veolicties's range
                if vLpossible <= self.MAX_VEL_LINEAR and vRpossible <= self.MAX_VEL_ANGULAR and vLpossible >= 0 and vRpossible >= -self.MAX_VEL_ANGULAR:
                    # Predict robot's new position in TAU seconds
                    predict_x, predict_y, predict_theta = self.predict_position(vLpossible, vRpossible, self.TAU)
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((self.x - goal[0]) ** 2 + (self.y - goal[1]) ** 2)
                    newTargetDistance = math.sqrt((predict_x - goal[0]) ** 2 + (predict_y - goal[1]) ** 2)
                    distanceForward = previousTargetDistance - newTargetDistance
                    # Cost term about distance to goal for evaluation
                    dist_goal_cost = self.FORWARDWEIGHT * distanceForward
                    # Cost term about distance to closest obstacle for evaluation
                    distanceToObstacle = self.calculateClosestObstacleDistance(predict_x, predict_y, obstacles)
                    if distanceToObstacle < self.SAFE_DIST:
                        dist_obstacle_cost = self.OBSTACLEWEIGHT * (self.SAFE_DIST - distanceToObstacle)
                    else:
                        dist_obstacle_cost = 0

                    # Total cost
                    benefit = dist_goal_cost - dist_obstacle_cost
                    if benefit > bestBenefit:
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit
        # Update velocities
        self.linear_vel = vLchosen
        self.angular_vel = vRchosen
        # print('[vLchosen:\t{:.3f}]\t[vRchosen:\t{:.3f}]'.format(vLchosen, vRchosen))

        # Return path to draw
        return vLchosen, vRchosen

