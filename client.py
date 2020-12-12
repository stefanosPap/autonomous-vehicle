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
    def __init__(self):
        self.client = None
        self.world = None
        self.map = None
        self.blueprint_library = None
        self.created_actor_list = []

    def connect(self):  
        self.client = carla.Client('localhost', 2000)                # create a client 
        self.client.set_timeout(10.0)                                # sets in seconds the maximum time a network call is allowed before blocking it     
        self.world = self.client.get_world()                         # returns the world object currently active in the simulation 
        self.map = self.world.get_map()                              # returns the map that we are working on. The object returned is of type carla.Map 
        self.blueprint_library = self.world.get_blueprint_library()  # returns a list of actor blueprints available to ease the spawn of these into the world
        self.settings = self.world.get_settings()
        self.settings.synchronous_mode = True
        self.settings.fixed_delta_seconds = 0.05
        self.world.apply_settings(self.settings)

    def add_actor(self, actor):
        self.created_actor_list.append(actor)
    
    def get_simulation(self):
        return [self.blueprint_library, self.world, self.map]

    def get_created_actors(self):
        return self.created_actor_list