#!/usr/bin/python3 
import glob
import os 
import sys 
import time 

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
        self.sensor = None


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

    def set_sensor(self, **kwargs):
        blueprint_sensor = self.blueprint.find(self.name)
        self.sensor = self.world.spawn_actor(blueprint_sensor, self.get_transform(), attach_to = self.get_vehicle())
        for key, value in kwargs.items():
            blueprint_sensor.set_attribute(str(key), str(value))

    def get_sensor(self):
        return self.sensor



class Lidar(Sensor):
    def __init__(self, name):
        super().__init__(name)
    
    def lidar_callback(self,point_cloud):
        print("Lidar measure:\n"+str(point_cloud)+'\n')
 
    def read(self):
        self.lidar = super().get_sensor()
        self.lidar.listen(lambda point_cloud: self.lidar_callback(point_cloud))



class CameraRGB(Sensor):
    def __init__(self, name):
        super().__init__(name)
    
    def image_callback(self, image):
        print("Camera measure:\n"+str(image)+'\n')
        
        
    def read(self):
        self.camera = super().get_sensor()
        self.camera.listen(lambda image: self.image_callback(image))


class GNSS(Sensor):
    def __init__(self, name):
        super().__init__(name)

    def gnss_callback(self, gnss):
        print("GNSS measure:\n"+str(gnss)+'\n')
        
    def read(self):
        self.gnss = super().get_sensor()
        self.gnss.listen(lambda gnss: self.gnss_callback(gnss))

class IMU(Sensor):
    def __init__(self, name):
        super().__init__(name)

    def imu_callback(self, imu):
        print("IMU measure:\n"+str(imu)+'\n')
        
    def read(self):
        self.imu = super().get_sensor()
        self.imu.listen(lambda imu: self.imu_callback(imu))