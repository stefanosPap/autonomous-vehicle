#!/usr/bin/python3 
import glob
import os 
import sys 
import time
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
import math
#execution_path = os.getcwd()


class Sensor(object):               
    """
    Description:
        Class Sensor is responsible for setting the correct settings to each specific sensor. It is the parent class of all the other sensor classes that implement specific sensors.

    """    

    def __init__(self, name):
        """
        Description:
            Method __init__ is the Constructor of Class Sensor that initializes most of the used variables 

        Args:
            name (str): The name of the sensor 
        """        

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
        """
        Description:
            Method set_location is used to set the location (coordinates) of the sensor  

        Args:
            X (float): The X coordinate
            Y (float): The Y coordinate
            Z (float): The Z coordinate
        """        

        self.locationX = X
        self.locationY = Y
        self.locationZ = Z
        self.location = carla.Location(X, Y, Z)
    

    def set_rotation(self, pitch, yaw, roll):
        """
        Description:
            Method set_location is used to set the rotation (orientation) of the sensor  

        Args:
            pitch (float): The Y-axis rotation angle
            yaw   (float): The Z-axis rotation angle 
            roll  (float): The X-axis rotation angle
        """        

        self.rotationRoll = roll
        self.rotationPitch = pitch
        self.rotationYaw = yaw
        self.rotation = carla.Rotation(pitch, yaw, roll) 


    def get_transform(self):
        """
        Description:
            Method get_transform is a getter that returns the transform of the sensor 

        Returns:
            carla.Transform: The sensor's transform
        """        

        #self.transform = carla.Transform(self.location, self.rotation)
        self.transform = carla.Transform()
        return self.transform


    def get_vehicle(self):
        """
        Description:
            Method get_vehicle is a getter that returns the vehicle that the sensor is attached to 

        Returns:
            carla.Vehicle: The vehicle that the sensor is attached to 
        """   

        return self.vehicle 


    def set_vehicle(self, vehicle):
        """
        Description:
            Method set_vehicle is a setter that sets the vehicle that the sensor will be attached to 

        Args:
            vehicle (carla.Vehicle): The vehicle that the sensor will be attached to 
        """        
        
        self.vehicle = vehicle


    def set_simulation(self, blueprint, world, map):
        """
        Description:
            Method set_simulation is a setter that sets the elements of the simulation  

        Args:
            blueprint       (carla.Blueprint)       :    Blueprint object of CARLA API
            world           (carla.World)           :    World object of CARLA API
            map             (carla.Map)             :    Map object of CARLA API
        """        

        self.world = world
        self.blueprint = blueprint
        self.map = map

    def set_sensor(self, **kwargs):
        """
        Description:
            Method set_sensor is responsible for setting the correct attributes to the chosen sensor 
        """       

        self.blueprint_sensor = self.blueprint.find(self.name)
        self.sensor = self.world.spawn_actor(self.blueprint_sensor, self.get_transform(), attach_to = self.get_vehicle())
        
        for key, value in kwargs.items():
            if  self.blueprint_sensor.has_attribute(str(key)):
                self.blueprint_sensor.set_attribute(str(key), str(value))
                print('{}'.format(str(key)), "set to", '{}'.format(str(value)))
            else:
                print('{}'.format(str(key)), "is not a valid attribute for", '{}'.format(self.name))


    def get_sensor(self):
        """
        Description:
            Method get_sensor is a getter that returns the sensor object

        Returns:
            carla.Sensor: The sensor object
        """   

        return self.sensor

# --------------------------------------- #
# |                                     | #
# |          Active Sensors             | #
# |                                     | #
# --------------------------------------- #

class Lidar(Sensor):
    """
    Description:
        Class Lidar implements the sensor of LiDAR

    Inherits:
        Parent class Sensor 
    """    

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class Lidar that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.lidar.ray_cast')
    

    def lidar_callback(self, point_cloud):    
        """
        Description:
            Method lidar_callback is used to handle lidar data when come to the sensor 

        Args:
            point_cloud (carla.LidarMeasurement): The measurement of the lidar in a specific timestamp
        """      

        #print("range: ", self.blueprint_sensor.get_attribute('range'))
        #print("points per second: ", self.blueprint_sensor.get_attribute('points_per_second'))
        #print("rotation frequency: ", self.blueprint_sensor.get_attribute('rotation_frequency'))
        #print("lower_fov: ", self.blueprint_sensor.get_attribute('lower_fov'))
        #print("upper_fov: ", self.blueprint_sensor.get_attribute('upper_fov'))
        #print("timestamp: ", point_cloud.timestamp)
        #print("channels: ", self.blueprint_sensor.get_attribute('channels'))
        pass 
    
    def read(self):
        """
        Description:
            Method read is used to read the measurements of the lidar. 
            This function is called once, but every time a new measurement is logged
            then read method calls lidar callback to handle the new measurement. 
        """ 

        self.lidar = super().get_sensor()
        self.lidar.listen(lambda point_cloud: self.lidar_callback(point_cloud))


class Radar(Sensor):
    """
    Description:
        Class Radar implements the sensor of Radar

    Inherits:
        Parent class Sensor 
    """  

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class Radar that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.other.radar')


    def radar_callback(self, radar):
        """
        Description:
            Method radar_callback is used to handle radar data when come to the sensor 

        Args:
            radar (carla.RadarMeasurement): The measurement of the radar in a specific timestamp
        """     

        points = np.frombuffer(radar.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (len(radar), 4))

        for detect in radar:
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            self.world.debug.draw_point(radar.transform.location + fw_vec, size=0.075, life_time=0.06, persistent_lines=False, color=carla.Color(10, 11, 10))
        
    def read(self):
        """
        Description:
            Method read is used to read the measurements of the radar. 
            This function is called once, but every time a new measurement is logged
            then read method calls radar callback to handle the new measurement. 
        """ 

        self.radar = super().get_sensor()
        self.radar.listen(lambda radar: self.radar_callback(radar))


# --------------------------------------- #
# |                                     | #
# |              Cameras                | #
# |                                     | #
# --------------------------------------- #

class CameraRGB(Sensor):
    """
    Description:
        Class CameraRGB implements the sensor of CameraRGB

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class CameraRGB that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.camera.rgb')
        #self.detector = ObjectDetection()
        #self.detector.setModelTypeAsYOLOv3()
        #self.detector.setModelPath( os.path.join(execution_path , "yolo.h5"))
        #self.detector.loadModel()


    def image_callback(self, image):  
        """
        Description:
            Method image_callback is used to handle image data when come to the camera 

        Args:
            image (carla.Image): The measurement of the camera in a specific timestamp
        """ 

        #image.save_to_disk('/home/stefanos/Desktop/dataCameraRGB/%.6d.jpg' % image.frame)
        #detections = self.detector.detectObjectsFromImage(input_image='/home/stefanos/Desktop/dataCameraRGB/%.6d.jpg' % image.frame, output_image_path=os.path.join(execution_path , '%.6d.jpg' % image.frame))
        pass

    def read(self):
        """
        Description:
            Method read is used to read the measurements of the camera. 
            This function is called once, but every time a new measurement is logged
            then read method calls image callback to handle the new measurement. 
        """ 

        self.camera = super().get_sensor()
        self.camera.listen(lambda image: self.image_callback(image))


class CameraSemantic(Sensor):
    """
    Description:
        Class CameraSemantic implements the sensor of CameraSemantic

    Inherits:
        Parent class Sensor 
    """ 
    
    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class CameraSemantic that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.camera.semantic_segmentation')

    def image_callback(self, image):
        """
        Description:
            Method image_callback is used to handle image data when come to the camera 

        Args:
            image (carla.Image): The measurement of the camera in a specific timestamp
        """ 

        pass
        
    def read(self):
        """
        Description:
            Method read is used to read the measurements of the camera. 
            This function is called once, but every time a new measurement is logged
            then read method calls image callback to handle the new measurement. 
        """ 

        self.camera = super().get_sensor()
        self.camera.listen(lambda image: self.image_callback(image))


# --------------------------------------- #
# |                                     | #
# |          Position Sensors           | #
# |                                     | #
# --------------------------------------- #

class GNSS(Sensor):
    """
    Description:
        Class GNSS implements the sensor of GNSS

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class GNSS that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.other.gnss')


    def gnss_callback(self, gnss):
        """
        Description:
            Method gnss_callback is used to handle gnss data when come to the sensor 

        Args:
            gnss (carla.GNSSMeasurement): The measurement of the GNSS in a specific timestamp
        """ 

        #print("GNSS measure:\n"+str(gnss)+'\n')
        pass


    def read(self):
        """
        Description:
            Method read is used to read the measurements of the sensor. 
            This function is called once, but every time a new measurement is logged
            then read method calls gnss callback to handle the new measurement. 
        """ 
        
        self.gnss = super().get_sensor()
        self.gnss.listen(lambda gnss: self.gnss_callback(gnss))


class IMU(Sensor):
    """
    Description:
        Class IMU implements the sensor of IMU

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class IMU that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.other.imu')


    def imu_callback(self, imu):
        """
        Description:
            Method imu_callback is used to handle imu data when come to the sensor 

        Args:
            imu (carla.IMUMeasurement): The measurement of the IMU in a specific timestamp
        """ 

        #print("IMU measure:\n"+str(imu)+'\n')
        pass
    

    def read(self):
        """
        Description:
            Method read is used to read the measurements of the camera. 
            This function is called once, but every time a new measurement is logged
            then read method calls imu callback to handle the new measurement. 
        """ 

        self.imu = super().get_sensor()
        self.imu.listen(lambda imu: self.imu_callback(imu))


# --------------------------------------- #
# |                                     | #
# |              Detectors              | #
# |                                     | #
# --------------------------------------- #

class ObstacleDetector(Sensor):
    """
    Description:
        Class ObstacleDetector implements the sensor of ObstacleDetector

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class ObstacleDetector that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        self.front_obstacle = False
        self.other_actor = None
        super().__init__('sensor.other.obstacle')


    def obstacle_callback(self, obs):
        """
        Description:
            Method obstacle_callback is used to handle obstacles data when come to the sensor 

        Args:
            obs (carla.ObstacleDetectionEvent): The measurement of the ObstacleDetector in a specific timestamp
        """ 

        #print("Obstacle Detector measure:\n" + str(obs.distance) + '\n')
        #if obs.distance < 5 and obs.distance != 0.0:
        self.front_obstacle = True
        self.other_actor = obs.other_actor
        #else:
        #    self.front_obstacle = False


    def read(self):
        """
        Description:
            Method read is used to read the measurements of the sensor. 
            This function is called once, but every time a new measurement is logged
            then read method calls obstacle callback to handle the new measurement. 
        """ 

        self.obstacle_detector = super().get_sensor()
        self.obstacle_detector.listen(lambda obs: self.obstacle_callback(obs))
    

    def get_front_obstacle(self):
        """
        Description:
            Method get_front_obstacle is a getter that returns the state of obstacle detection

        Returns:
            boolen: The state of obstacle detection (True if obstacle detected)
        """   

        return self.front_obstacle


    def set_front_obstacle(self, state):
        """
        Description:
            Method set_front_obstacle is responsible for setting the state of obstacle detection 
        """  

        self.front_obstacle = state


    def get_other_actor(self):
        """
        Description:
            Method get_other_actor is a getter that returns the actor of the obstacle detected 

        Returns:
            carla.Vehicle: The obstacle object
        """  

        return self.other_actor


class LaneInvasionDetector(Sensor):
    """
    Description:
        Class LaneInvasionDetector implements the sensor of LaneInvasionDetector

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class LaneInvasionDetector that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.other.lane_invasion')


    def lane_callback(self, lane):
        """
        Description:
            Method lane_callback is used to handle lane data when come to the sensor 

        Args:
            lane (carla.LaneInvasionEvent): The measurement of the LaneInvasionDetector in a specific timestamp
        """ 

        #print("Lane Invasion Detector measure:\n"+str(lane)+'\n')
        pass 
    
    
    def read(self):
        """
        Description:
            Method read is used to read the measurements of the sensor. 
            This function is called once, but every time a new measurement is logged
            then read method calls lane callback to handle the new measurement. 
        """ 

        self.lane_detector = super().get_sensor()
        self.lane_detector.listen(lambda lane: self.lane_callback(lane))


class CollisionDetector(Sensor):
    """
    Description:
        Class CollisionDetector implements the sensor of CollisionDetector

    Inherits:
        Parent class Sensor 
    """ 

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class CollisionDetector that initializes the name of the sensor. The rest of the variables are initialized through the parent class. 
        """

        super().__init__('sensor.other.collision')


    def collision_callback(self, collision):
        """
        Description:
            Method collision_callback is used to handle lane data when come to the sensor 

        Args:
            collision (carla.CollisionEvent): The measurement of the CollisionDetector in a specific timestamp
        """ 

        print("Collision Detector measure:\n"+str(collision.other_actor)+'\n')
        return True 


    def read(self):
        """
        Description:
            Method read is used to read the measurements of the sensor. 
            This function is called once, but every time a new measurement is logged
            then read method calls collision callback to handle the new measurement. 
        """ 

        self.collision_detector = super().get_sensor()
        self.collision_detector.listen(lambda collision: self.collision_callback(collision))
