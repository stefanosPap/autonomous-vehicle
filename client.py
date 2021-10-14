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

class Client(object):
    """
    Description:
        Class Client implements a carla Client that contains all the necessary information and methods that needed to connect to the simulation server  
    """    

    def __init__(self):
        """
        Description:
            Method __init__ is the Constructor of Class Client that initializes most of the used variables 
        """        

        self.client = None
        self.world = None
        self.map = None
        self.blueprint_library = None
        self.created_actor_list = []


    def connect(self):  
        """
        Description:
            Method connect is responsible for setting the most important settings of the connection and to connect the client with the simulation server 
        """        

        self.client = carla.Client('localhost', 2000)                # create a client 
        self.client.set_timeout(10.0)                                # sets in seconds the maximum time a network call is allowed before blocking it     
        self.world = self.client.get_world()                         # returns the world object currently active in the simulation 
        self.map = self.world.get_map()                              # returns the map that we are working on. The object returned is of type carla.Map 
        self.blueprint_library = self.world.get_blueprint_library()  # returns a list of actor blueprints available to ease the spawn of these into the world
        self.settings = self.world.get_settings()
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(self.settings)


    def get_client(self):
        """
        Description:
            Method get_client is a getter that returns the specified client object 

        Returns:
            carla.Client: Client object 
        """        
    
        return self.client
        

    def add_actor(self, actor):
        """
        Description:
            Method add_actor adds an actor to the overall actors list 

        Args:
            actor (carla.Actor): An actor that is added to the overall actors list 
        """        

        self.created_actor_list.append(actor)
    

    def get_simulation(self):
        """
        Description:
            Method get_simulation is a getter that returns a list with the specified simulation objects 

        Returns:
            list: simulation objects (blueprint library, world, map)
        """ 

        return [self.blueprint_library, self.world, self.map]


    def get_created_actors(self):
        """
        Description:
            Method get_created_actors is a getter that returns the list with the actors

        Returns:
            list: The created actors  
        """ 

        return self.created_actor_list


    def get_maps(self):
        """
        Description:
            Method get_maps is a getter that returns the available maps of the CARLA API

        Returns:
            list: The available maps of the CARLA API
        """ 

        return self.client.get_available_maps()


    def set_map(self, map):
        """
        Description:
            Method set_map is used to load a specific map in the simulation

        Args:
            map (carla.Map): Map object that is loaded to the simulation 
        """        

        self.client.load_world(map)
