#!/usr/bin/python3 
import glob
import os 
import sys 

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla 

class Sensor(object):

    def __init__(self,name):
        self.name = name
        self.locationX = 0
        self.locationY = 0
        self.locationZ = 0
        self.rotationRoll= 0
        self.rotationPitch= 0
        self.rotationYaw = 0

    def set_location(self, X, Y, Z):
        self.locationX = X
        self.locationY = Y
        self.locationZ = Z
        self.location = carla.Location(X, Y, Z)
    
    def set_rotation(self, pitch, yaw, roll):
        self.rotationRoll = roll
        self.rotationPitch = pitch
        self.rotationYaw = yaw
        self.rotation = carla.Rotation(pitch, yaw, roll) 

    def get_transform(self):
        self.transform = carla.Transform(self.location, self.rotation)
        return self.transform

    def get_vehicle(self):
        return self.vehicle 

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def set_simulation(self, blueprint, world, map):
        self.world = world
        self.blueprint = blueprint
        self.map = map

    def set_sensor(self, name):
        blueprint_sensor = self.blueprint.find(name)
        self.sensor = self.world.spawn_actor(blueprint_sensor, self.get_transform(), attach_to = self.get_vehicle())
        
    def get_sensor(self):
        return self.sensor
        
class Lidar(Sensor):

    def __init__(self, name, X, Y, Z, roll, pitch, yaw):
        super().__init__(name)
        super().set_location(X, Y, Z)
        super().set_rotation(pitch, yaw, roll)
        super().set_sensor(name)
        #lidar.listen(lambda point_cloud: point_cloud.save_to_disk('/home/stefanos/Desktop/data/%.6d.ply' % point_cloud.frame))
