#!/usr/bin/env python3

import rospy
import os
import random
import time

import numpy as np

from airsim import MultirotorClient
from airsim.types import Pose
from airsim.utils import to_quaternion, to_eularian_angles

from std_msgs.msg import String

from .utils import pose, angular_distance


class Ue4Briedge:
    """Starts communication's Engine.

    Args:
        HOST: 
            -For set ip address from host, use ipconfig result's on host
            -For set ip address from docker network, use a ip from ping result's between containers on host
            -For set ip address from WSL, os.environ['WSL_HOST_IP'] on host.
    """        
    
    def __init__(self) -> None:
        self.client = MultirotorClient(os.environ['UE4_IP'])
        self.client.confirmConnection()
        rospy.logwarn(f"\nConnection: {self.client.ping()}")     
        self.spawn_resources = _SpawResources(self.client)
        
    def restart(self) -> None:
        """
        Reset the ue4 client conection.
        """        
        rospy.logwarn(f"\nRestart Connection: {self.client.ping()}")
        self.client.reset()
    
    
class _SpawResources:
    
    def __init__(self, ue4_connection : MultirotorClient):
        self.__conn = ue4_connection
        self.__pub_info = rospy.Publisher("simulation_info", String, queue_size=10)
        
    def _get_object_pose(self, object_name : str) -> Pose:
        """Returns a scene element pose.

        Args:
            object_name (str): Element's name.

        Returns:
            Pose: AirSim native pose.
        """        
        pose = self.__conn.simGetObjectPose(object_name)
        return pose
    
    def _get_vehicle_pose(self, vehicle_name : str) -> Pose:
        """Returns a scene element pose.

        Args:
            vehicle_name (str): Vehicle's name.

        Returns:
            Pose: AirSim native pose.
        """        
        pose = self.__conn.simGetVehiclePose(vehicle_name)
        return pose
            
    def _set_vehicle_pose(self, vehicle_name : str, position : tuple, eularian_orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one vehicle.

        Args:
            vehicle_name (str): The same was defined on settings.json .
            position (tuple): (x,y,z).
            eularian_orientation (tuple): (pitch, roll, yaw).
        """           
        pose = pose(position, eularian_orientation)
        self.__conn.simSetVehiclePose(pose, True, vehicle_name)
        
        if debbug:
            info = f"New Vehicle pose was defined: {vehicle_name} - [{position}, {eularian_orientation}]"
            self.__pub_info.publish(info) 
            
    def _set_object_pose(self, object_name : str, position : tuple, eularian_orientation : tuple, debbug : bool = False) -> None:
        """Define a new pose at one scene object.

        Args:
            object_name (str): Element's name.
            position (tuple): (x,y,z).
            eularian_orientation (tuple): (pitch, roll, yaw).
        """           
        pose = pose(position, eularian_orientation)
        self.__conn.simSetObjectPose(pose, True, object_name)
        
        if debbug:
            info = f"New Object pose was defined: {object_name} - [{position}, {eularian_orientation}]"
            self.__pub_info.publish(info) 

    def _air_random_circular_pose(self, vehicle_name : str, target : str, radius : float, dist : float) -> tuple:
        """Define a random vehicle pose at target based on radius range and secure distance to avoid collision.

        Args:
            radius (float): Range around the target to define vehicle's pose.
            dist (float): Secure distance to spawn.
            target (str): Scene object name.

        Returns:
            tuple: (new x, new y, new yaw)
        """        
        _, _, current_yaw = self.get_current_eularian_vehicle_angles(vehicle_name)
        pose = self._get_object_pose(target)

        x = pose.position.x_val
        y = pose.position.y_val
        
        b_supx = x - dist
        b_infx = b_supx - radius
        
        a_supx = x + dist
        a_infx = b_supx + radius
        
        b_supy = y - dist
        b_infy = b_supy - radius
        
        a_supy = y + dist
        a_infy = a_supy + radius

    
        
        nx = random.choice(np.hstack((np.arange(b_infx, b_supx, 0.5), np.arange(a_infx, a_supx, 0.5))))
        ny = random.choice(np.hstack((np.arange(b_infy, b_supy, 0.5), np.arange(a_infy, a_supy, 0.5))))

        nyaw = current_yaw + angular_distance((nx, ny, 0), (x, y, 0), degrees=True)

        return (nx, ny, nyaw)
          
    def get_current_eularian_vehicle_angles(self, vehicle_name : str) -> tuple:
        """Get current eularian angles from vehicle.

        Args:
            vehicle_name (str): The same was defined on settings.json .

        Returns:
            tuple: (pitch, roll, yaw)
        """        
        pose = self._get_vehicle_pose(vehicle_name)

        pitch, roll, yaw = to_eularian_angles(pose.orientation)
        return (pitch, roll, yaw)
    
    def set_air_random_circular_pose(self, vehicle_name : str, target : str, radius : float, dist : float) -> None:
        """Set a random vehicle pose at target based on radius range and secure distance to avoid collision.

        Args:
            radius (float): Range around the target to define vehicle's pose.
            dist (float): Secure distance to spawn.
            target (str): Scene object name.

        Returns:
            tuple: (new x, new y, new yaw)
        """   
        pose = self._get_vehicle_pose(vehicle_name)
        z = pose.position.z_val

        x, y, yaw = self._air_random_circular_pose(vehicle_name, target, radius, dist)
        position = (x, y, z)
        eularian_orientation = (0, 0, yaw)

        self._set_vehicle_pose(vehicle_name, position, eularian_orientation)
 


    