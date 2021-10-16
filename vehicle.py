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

class Vehicle(object):
    """
    Description:
        Class Vehicle is used to create a single vehicle in the map of CARLA by setting all the appropriate settings in order to work properly
    """    

    def __init__(self):    
        """
            Method __init__ is the Constructor of Class Vehicle that initializes most of the used variables 
        """        

        self.vehicle_model = None
        self.vehicle_actor = None
        self.start_point = None 
        self.sensors = []


    def choose_model(self, model, bp, world):
        """
        Description:
            Method choose_model is used to choose the model of the vehicle that will be cretaed and to spawn the vehicle in the scene 

        Args:
            model       (str)                   :   The name of the model
            bp          (carla.Blueprint)       :   Blueprint object of CARLA API
            world       (carla.World)           :   World object of CARLA API
        """    

        self.blueprint_library = bp
        self.world = world  
        self.vehicle_model = self.blueprint_library.filter(model)[0]                        # get vehicle (tesla model3 from library)
        self.vehicle_actor = self.world.spawn_actor(self.vehicle_model, self.start_point)   # spawn the vehicle to the first point    
    

    def choose_spawn_point(self, point):
        # set the spawn point 
        """
        Description:
            Method choose_spawn_point is used to choose the point where the vehicle will be spawned 

        Args:
            point (carla.Location): The location where the vehicle will be spawned 
        """

        self.start_point = point


    def wander(self):
        """
        Description:
            Method wander is used to set the vehicle in wandering mode by applying the autopilot that is provided by the CARLA
        """        
  
        while True:
            try:
                self.vehicle_actor.set_autopilot(True) 
                #self.set_spectator()
                self.world.tick()                                          # infinite loop in order not to terminate the programm  

            except KeyboardInterrupt:
                break


    def get_vehicle_actor(self):
        """
        Description:
            Method get_vehicle_actor is a getter that returns the specified vehicle actor object 

        Returns:
            carla.Vehicle: Vehicle object 
        """    

        return self.vehicle_actor


    def get_vehicle_transform(self):
        """
        Description:
            Method get_vehicle_transform is a getter that returns the specified vehicle's transform 

        Returns:
            carla.Transform:  The vehicle's transform 
        """  

        return self.vehicle_actor.get_transform()
        

    def add_sensor(self, sensor):
        """
        Description:
            Method add_sensor is used to add a created sensor to the vehicle's sensor list 

        Args:
            sensor (sensor.Sensor): The sensor that will be added 
        """        
        self.sensors.append(sensor)
        

    def set_spectator(self): 
        """
        Description:
            Method set_spectator is used to set the spectator of a vehicle properly in order to view the vehicle from the desired angle 
        """        
        
        spectator = self.world.get_spectator()                              # set a spectator      
        transform = self.get_vehicle_transform()                            # get the coordinates of the car in order to attach the vehicle  
        transform.location.z += 50                                          # increase z by 2 in order to place it on top of the car ??? This maybe should change ???
        transform.rotation.roll = 0         
        transform.rotation.pitch = -90
        transform.rotation.yaw = 180
        spectator.set_transform(transform)