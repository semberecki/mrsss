#!/usr/bin/env python2
"""
Custom TSP planner
@author: R.Penicka, T.Baca
"""

import rospy
import os
import copy
import numpy as np
import dubins
this_script_path = os.path.dirname(__file__)
this_script_path = os.path.dirname(__file__)
# MRS ROS messages
from mtsp_msgs.msg import TspProblem
from mrs_msgs.msg import TrackerTrajectory

# the TSP problem class
from mtsp_problem_loader.tsp_problem import *

from solvers.tsp_solvers import *
import solvers.tsp_trajectory

import matplotlib.pyplot as plt
from matplotlib.pyplot import cm

import argparse


class TspPlanner:

    def __init__(self, use_ros=True):

        if use_ros:
            print("runing with ros")

            rospy.init_node('tsp_planner', anonymous=True)

            self._max_velocity = rospy.get_param('~max_velocity', 4)
            self._max_acceleration = rospy.get_param('~max_acceleration', 2)
            self._turning_velocity = rospy.get_param('~turning_velocity', 2)
            self._plot = rospy.get_param('~plot', False)
    
            print("using max_velocity", self._max_velocity)
            print("using max_acceleration", self._max_acceleration)
            print("using plot", self._plot)
            print("using turning_velocity", self._turning_velocity)
    
            # based on the velocity and acceleration
            self.turning_radius = (self._max_velocity * self._max_velocity) / self._max_acceleration
    
            # initiate ROS publishers
            self.publisher_trajectory_1 = rospy.Publisher("~trajectory_1_out", TrackerTrajectory, queue_size=1)
            self.publisher_trajectory_2 = rospy.Publisher("~trajectory_2_out", TrackerTrajectory, queue_size=1)

            rate = rospy.Rate(1.0)
            rate.sleep()
    
            # initiate ROS subscribers
            self.subscriber_problem = rospy.Subscriber("~problem_in", TspProblem, self.callbackProblem, queue_size=1)
    
            rospy.loginfo('Planner initialized')

            rospy.spin()
            
        else:
            print("runing as bare script")
            config_file = os.path.join(this_script_path, "../config/simulation.yaml")
            with open(config_file, 'r') as f:
                loaded_config = yaml.safe_load(f)
                print("using default params from config file", config_file, ":", loaded_config)
            
            arg_parser = argparse.ArgumentParser(description='MTSP planner')
            arg_parser.add_argument('--max_velocity', type=float, default=loaded_config['max_velocity'])
            arg_parser.add_argument('--max_acceleration', type=float, default=loaded_config['max_acceleration'])
            arg_parser.add_argument('--turning_velocity', type=float, default=loaded_config['turning_velocity'])
            arg_parser.add_argument('--plot', type=bool, default=True)
            arg_parser.add_argument('--problem', type=str, default=os.path.join(this_script_path, "../../mtsp_problem_loader/problems/random_problem.tsp"))
            
            args = arg_parser.parse_args()
            print("running with args", args)
            
            self._max_velocity = args.max_velocity
            self._max_acceleration = args.max_acceleration
            self._plot = args.plot
            self._turning_velocity = args.turning_velocity
            
            print("using max_velocity", self._max_velocity)
            print("using max_acceleration", self._max_acceleration)
            print("using plot", self._plot)
            print("using turning_velocity", self._turning_velocity)
            
            # self._turning_radius = (self._max_velocity*self._max_velocity)/self._max_acceleration
            
            # load_problem
            tsp_problem = MTSPProblem.load_problem(args.problem) 
            
            self.plan_trajectory(tsp_problem)

    def callbackProblem(self, problem):
        """ros callback for mtsp problem message"""
        rospy.loginfo_throttle(1.0, 'Received the TSP problem')

        # copy the points
        targets = []
        starts = []
        for point in problem.points:
            target = [point.idx, point.x, point.y]
            targets.append(target)
        for point in problem.start_points:
            start = [point.idx, point.x, point.y]
            starts.append(start)

        # create the python tsp_problem object
        tsp_problem = MTSPProblem(problem.name, problem.type, problem.comment, problem.dimension, problem.number_of_robots, starts, problem.neighborhood_radius, problem.edge_weight_type, targets)

        rospy.loginfo('The problem is {}'.format(problem))

        trajectories_samples = self.plan_trajectory(tsp_problem)

        # # | ------------------ plot velocity profile ----------------- |
        # plt.show()

        # # | --------------- create the ROS trajectories -------------- |
        trajectory = solvers.tsp_trajectory.TSPTrajectory(self._max_velocity, self._max_acceleration)
        ros_trajectory_1 = trajectory.create_ros_trajectory(trajectories_samples[0], problem.height)
        ros_trajectory_2 = trajectory.create_ros_trajectory(trajectories_samples[1], problem.height)

        # # | ---------------- publish the trajectories ---------------- |
        self.publisher_trajectory_1.publish(ros_trajectory_1)
        self.publisher_trajectory_2.publish(ros_trajectory_2)

        rospy.loginfo('trajectories were published')

    def get_cluster_direction(self, full_cluster):
        start = full_cluster[0][1:]
        first=full_cluster[1][1:]
        last=full_cluster[-1][1:]

        a= first[0]-start[0]
        b=first[1]-start[1]
        c=last[0]-start[0]
        d=last[1]-start[1]
        dist1 = a**2+b**2
        dist2 = c**2+ d**2
        if dist1>dist2:
            #a=  full_cluster[0:0] + list(reversed(full_cluster[1:]))
            a=  [full_cluster[0]]
            a.extend(list(reversed(full_cluster[1:])))
            return a
        else:
            return full_cluster

    def calc_angle(self, first, second):
        x1 = first[0]
        y1 = first[1]
        x2 = second[0]
        y2 = second[1]
        return math.atan2(y2 - y1, x2 - x1)

    def plan_trajectory(self, tsp_problem): 
        """method for planning the M(D)TSP(N) plans based on tsp_problem"""
        
        # copy the points
        if self._plot:
            ax = MTSPProblem.plot_problem(tsp_problem, show=False)
            arena_corners = read_world_file_arena(os.path.join(this_script_path, "../../mtsp_state_machine/config/simulation/world.yaml"))
            plt.plot([arena_corners[i][0] for i in range(-1, len(arena_corners))], [arena_corners[i][1] for i in range(-1, len(arena_corners))], 'c-',label='fly area')
            
        tsp_solver = TSPSolver()
        
        
        ############### TARGET LOCATIONS CLUSTERING BEGIN ###############
        clusters = [[tsp_problem.start_positions[i]] for i in range(tsp_problem.number_of_robots)]  # initiate cluster with starts
        # for i in range(tsp_problem.number_of_robots):
        #     start_id = i * len(tsp_problem.targets) / tsp_problem.number_of_robots
        #     stop_id = (i + 1) * len(tsp_problem.targets) / tsp_problem.number_of_robots
        #     old[i] += tsp_problem.targets[start_id:stop_id]

        kmeans_clusters, centroid= tsp_solver.cluster_kmeans(tsp_problem.targets, tsp_problem.number_of_robots)

        tsp_solver.cluster_kmeans(tsp_problem.targets, tsp_problem.number_of_robots)
        print("kmeans_clusters", kmeans_clusters)

        cluster1_distance1 = (clusters[0][0][1] -float(list(centroid[0])[0]))**2 + (clusters[0][0][2] -float(list(centroid[0])[1]))**2
        cluster1_distance2 = (clusters[0][0][1] -float(list(centroid[1])[0]))**2 + (clusters[0][0][2] -float(list(centroid[1])[1]))**2
        print("cluster1_distance1", cluster1_distance1, cluster1_distance2)

        if cluster1_distance1 > cluster1_distance2:
            clusters[0] += kmeans_clusters[1]
            clusters[1] += kmeans_clusters[0]

        else:
            clusters[0] += kmeans_clusters[0]
            clusters[1] += kmeans_clusters[1]

        # for i in range(tsp_problem.number_of_robots):
        #     clusters[i] = self.get_cluster_direction(clusters[i])
            #clusters[i][0:] + list(reversed(clusters[i][1:]))
        # clusters[0] =clusters[0][0:] + list(reversed(clusters[0][1:]))
        # clusters[1] =clusters[1][0:] + list(reversed(clusters[1][1:]))


        ############### TARGET LOCATIONS CLUSTERING END ###############
        
        
        # # | -------------------- plot the clusters ------------------- |
        if self._plot:  # plot the clusters
            colors = cm.rainbow(np.linspace(0, 1, tsp_problem.number_of_robots))
            for i in range(tsp_problem.number_of_robots):
                # plt.plot([cluster_centers[i][0]],[cluster_centers[i][1]],'*',color=colors[i])
                plt.plot([c[1] for c in clusters[i]], [c[2] for c in clusters[i]], '.', color=colors[i])

        # # | ---------------------- solve the TSP --------------------- |
        robot_sequences = []
        # for i in range(1):
        for i in range(tsp_problem.number_of_robots):

            ############### TSP SOLVERS PART BEGIN ###############
            #path = tsp_solver.plan_tour_etsp(clusters[i],0) #find decoupled ETSP tour over clusters
            #path = tsp_solver.plan_tour_etspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.65)  # find decoupled ETSPN tour over clusters
            
            turning_radius = (self._turning_velocity * self._turning_velocity) / self._max_acceleration
            path = tsp_solver.plan_tour_dtspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.62, turning_radius)  # find decoupled DTSPN tour over clusters
            #path = tsp_solver.plan_tour_dtspn_noon_bean(clusters[i], 0, tsp_problem.neighborhood_radius *0.5, turning_radius) # find noon-bean DTSPN tour over clusters


            #path = path[:-1]
            ############### TSP SOLVERS PART END ###############

            a = path[1][0] - path[0][0]
            b = path[1][1] - path[0][1]
            c = path[-1][1] - path[0][1]
            d = path[-1][1] - path[0][1]
            dist1 = a ** 2 + b ** 2
            dist2 = c ** 2 + d ** 2



            if dist1 > dist2:
                new_path = []
                for p in path:
                    new_path.append((p[0], p[1],  p[2] + math.pi))
                path = new_path[::-1]
                path = path[:-1]
                path[-1] = (path[-1][0], path[-1][1],   self.calc_angle( path[-2][:2],  path[-1][:2]))
            else:
                path = path[:-1]

            robot_sequences.append(path)
            
            # # | -------------------- plot the solution ------------------- |
            if self._plot:  # plot tsp solution
                sampled_path_all = []
                for pid in range(1, len(path)):
                    if len(path[pid]) == 2 :
                        sampled_path_all += path
                        sampled_path_all += [path[0]]
                        #plt.plot([path[pid - 1][0] , path[pid][0]], [path[pid - 1][1] , path[pid][1]], '-', color=colors[i], lw=0.8, label='trajectory %d' % (i + 1))
                    elif len(path[pid]) == 3 :
                        dubins_path = dubins.shortest_path(path[pid - 1], path[pid], turning_radius)
                        sampled_path , _ = dubins_path.sample_many(0.1)
                        sampled_path_all += sampled_path
                
                plt.plot([p[0] for p in sampled_path_all] , [p[1] for p in sampled_path_all] , '-', color=colors[i], lw=1.2, label='trajectory %d' % (i + 1))

        trajectory = tsp_trajectory.TSPTrajectory(self._max_velocity, self._max_acceleration)

        # # | ------------------- sample trajectories ------------------ |
        trajectories_samples = []
        max_trajectory_time = 0
        for i in range(len(robot_sequences)):

            if len(robot_sequences[i][0]) == 2 :
                single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_euclidean(robot_sequences[i])
            elif len(robot_sequences[i][0]) == 3:
                single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_dubins(robot_sequences[i], turning_velocity=self._turning_velocity)
            
            print("trajectory_time", i, "is", trajectory_time)
            trajectories_samples.append(single_trajectory_samples)
            
            if trajectory_time > max_trajectory_time:
                max_trajectory_time = trajectory_time
            
            if self._plot:  # plot trajectory samples
                plt.plot([p[0] for p in single_trajectory_samples], [p[1] for p in single_trajectory_samples], 'o', markerfacecolor=colors[i], markeredgecolor='k', ms=2.2, markeredgewidth=0.4 , label='samples %d' % (i + 1))
                
        if self._plot:  # add legend to trajectory plot
            plt.legend(loc='upper right')
            
        print("maximal time of trajectory is", max_trajectory_time)

        # # | --------------- plot velocity profiles --------------- |
        if self._plot:  # plot velocity profile
            for i in range(len(trajectories_samples)):
                trajectory.plot_velocity_profile(trajectories_samples[i], color=colors[i],title = 'Velocity profile %d' % (i + 1))
        
        # # | ----------------------- show plots ---------------------- |
        if self._plot:
            plt.show()
        return trajectories_samples   
            

if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    if "--ros" in myargv:
        try:
            tsp_planner = TspPlanner()
        except rospy.ROSInterruptException:
            pass
    else:
        tsp_planner = TspPlanner(use_ros=False)
        
