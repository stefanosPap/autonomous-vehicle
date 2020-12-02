#!/usr/bin/python3 
import glob
import os 
import sys 
import time
import cv2
import numpy as np   
#from imageai.Detection import ObjectDetection

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
execution_path = os.getcwd()


class Sensor(object):               # general sensor class 

    def __init__(self,name):
        self.name = name
        self.locationX = 0
        self.locationY = 0
        self.locationZ = 0
        self.rotationRoll= 0
        self.rotationPitch= 0
        self.rotationYaw = 0
        self.sensor = None
        self.vehicle = None

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
        self.blueprint_sensor = self.blueprint.find(self.name)
        self.sensor = self.world.spawn_actor(self.blueprint_sensor, self.get_transform(), attach_to = self.get_vehicle())
        for key, value in kwargs.items():
            self.blueprint_sensor.set_attribute(str(key), str(value))

    def get_sensor(self):
        return self.sensor

###########################################
#----------------Sensors----------------###
###########################################

class Lidar(Sensor):
    def __init__(self):
        super().__init__('sensor.lidar.ray_cast')
    
    def lidar_callback(self,point_cloud):    
        print("range: ", self.blueprint_sensor.get_attribute('range'))
        print("points per second: ", self.blueprint_sensor.get_attribute('points_per_second'))
        print("rotation frequency: ", self.blueprint_sensor.get_attribute('rotation_frequency'))
        print("lower_fov: ", self.blueprint_sensor.get_attribute('lower_fov'))
        print("upper_fov: ", self.blueprint_sensor.get_attribute('upper_fov'))
        print("timestamp: ", point_cloud.timestamp)

    def read(self):
        self.lidar = super().get_sensor()
        self.lidar.listen(lambda point_cloud: self.lidar_callback(point_cloud))

class Radar(Sensor):
    def __init__(self):
        super().__init__('sensor.other.radar')

    def radar_callback(self, radar):
        print("Radar: ", radar.raw_data)
        print("timestamp: ", radar.timestamp)
        pass 
    def read(self):
        self.radar = super().get_sensor()
        self.radar.listen(lambda radar: self.radar_callback(radar))

###########################################
#----------------Cameras----------------###
###########################################

class CameraRGB(Sensor):
    def __init__(self):
        super().__init__('sensor.camera.rgb')
        #self.detector = ObjectDetection()
        #self.detector.setModelTypeAsYOLOv3()
        #self.detector.setModelPath( os.path.join(execution_path , "yolo.h5"))
        #self.detector.loadModel()

    def image_callback(self, image):
        #image.save_to_disk('/home/stefanos/Desktop/dataCameraRGB/%.6d.jpg' % image.frame)
           #detections = self.detector.detectObjectsFromImage(input_image='/home/stefanos/Desktop/dataCameraRGB/%.6d.jpg' % image.frame, output_image_path=os.path.join(execution_path , '%.6d.jpg' % image.frame))
        pass

    def process_img(self, image):
        i = np.array(image.raw_data)
        i2 = i.reshape((800, 600, 4))
        i3 = i2[:, :, :3]
        cv2.imshow("", i3)
        cv2.waitKey(0)
        

    def read(self):
        self.camera = super().get_sensor()
        self.camera.listen(lambda image: self.image_callback(image))


class CameraSemantic(Sensor):
    def __init__(self):
        super().__init__('sensor.camera.semantic_segmentation')

    def image_callback(self, image):
        #image.save_to_disk('/home/stefanos/Desktop/dataCameraSemanticSegmentation/%.6d.jpg' % image.frame, carla.ColorConverter.CityScapesPalette)
        pass
        
    def read(self):
        self.camera = super().get_sensor()
        self.camera.listen(lambda image: self.image_callback(image))

###########################################
#-----------------Position--------------###
###########################################

class GNSS(Sensor):
    def __init__(self):
        super().__init__('sensor.other.gnss')

    def gnss_callback(self, gnss):
        print("GNSS measure:\n"+str(gnss)+'\n')
        pass
    def read(self):
        self.gnss = super().get_sensor()
        self.gnss.listen(lambda gnss: self.gnss_callback(gnss))

class IMU(Sensor):
    def __init__(self):
        super().__init__('sensor.other.imu')

    def imu_callback(self, imu):
        print("IMU measure:\n"+str(imu)+'\n')
        pass
    def read(self):
        self.imu = super().get_sensor()
        self.imu.listen(lambda imu: self.imu_callback(imu))

###########################################
#---------------Detectors---------------###
###########################################

class ObstacleDetector(Sensor):
    def __init__(self):
        super().__init__('sensor.other.obstacle')

    def obstacle_callback(self, obs):
        #print("Obstacle Detector measure:\n"+str(obs)+'\n')
        pass
    def read(self):
        self.obstacle_detector = super().get_sensor()
        self.obstacle_detector.listen(lambda obs: self.obstacle_callback(obs))

class LaneInvasionDetector(Sensor):
    def __init__(self):
        super().__init__('sensor.other.lane_invasion')

    def lane_callback(self, lane):
        #print("Lane Invasion Detector measure:\n"+str(lane)+'\n')
        pass 
    def read(self):
        self.lane_detector = super().get_sensor()
        self.lane_detector.listen(lambda lane: self.lane_callback(lane))

class CollisionDetector(Sensor):
    def __init__(self):
        super().__init__('sensor.other.collision')

    def collision_callback(self, lane):
        #print("Collision Detector measure:\n"+str(lane)+'\n')
        pass 
    def read(self):
        self.collision_detector = super().get_sensor()
        self.collision_detector.listen(lambda collision: self.collision_callback(collision))

