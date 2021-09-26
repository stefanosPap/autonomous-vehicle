import carla
import numpy as np 

class WalkerManager(object):
    
    def __init__(self, map, vehicle_actor, walker_list, world):
        self.map   = map
        self.world = world
        
        self.vehicle_actor = vehicle_actor
        self.walker_list   = walker_list
        
        self.closest_front_right_walker      = None
        self.closest_front_left_walker       = None
        self.closest_front_walker            = None
        

        self.closest_distance_from_front_right_walker = float('inf')
        self.closest_distance_from_front_left_walker  = float('inf')
        self.closest_distance_from_front_walker       = float('inf')
        
        self.walkers_in_right_lane  = []
        self.walkers_in_left_lane   = []
        self.walkers_in_same_lane   = []

    def check_pedestrians(self, waypoints, index):

            self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
            self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)

            self.ego_vehicle = self.vehicle_actor

            self.walkers_in_right_lane = []
            self.walkers_in_left_lane  = []
            self.walkers_in_same_lane  = []
            
            self.front_right_walkers   = []
            self.front_left_walkers    = []
            self.front_walkers         = []
            
            self.distances_walkers_in_right_lane = []
            self.distances_walkers_in_left_lane  = []
            self.distances_walkers_in_same_lane  = []

            self.front_right_distances = []
            self.front_left_distances  = []
            self.front_distances       = []
            
            self.closest_distance_from_front_right_walker = float('inf')
            self.closest_distance_from_front_left_walker  = float('inf')
            self.closest_distance_from_front_walker       = float('inf')
            
            self.closest_front_right_walker = None
            self.closest_front_left_walker  = None
            self.closest_front_walker       = None
            
            for walker in self.walker_list:

                ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                other_waypoint = self.map.get_waypoint(walker          .get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                
                if other_waypoint == None:
                    continue
                
                # check if right lane exists 
                if waypoints[index].get_right_lane() is not None:
                    right_lane_id = waypoints[index].get_right_lane().lane_id

                else:
                    right_lane_id = None

                # check if left lane exists 
                if waypoints[index].get_left_lane() is not None:
                    left_lane_id = waypoints[index].get_left_lane().lane_id

                else:
                    left_lane_id = None

                # check if same lane exists 
                if waypoints[index] is not None:
                    same_lane_id = waypoints[index].lane_id

                else:
                    same_lane_id = None

                # check for side obstacles 
                if right_lane_id == other_waypoint.lane_id: 
                    self.walkers_in_right_lane.append(walker)

                if left_lane_id == other_waypoint.lane_id:
                    self.walkers_in_left_lane.append(walker)

                if same_lane_id == other_waypoint.lane_id:
                    self.walkers_in_same_lane.append(walker)

            for walker in self.walkers_in_right_lane:
                if not walker.is_alive:
                    self.front_right_walkers = []
                    self.front_right_distances = []
                    continue
                walker_location = walker.get_location()

                ego_distance_front_right = walker_location.distance(self.front_location)
                ego_distance_rear_right  = walker_location.distance(self.rear_location)
                ego_distance             = walker_location.distance(self.ego_vehicle.get_location())

                if ego_distance_front_right < ego_distance_rear_right:
                    self.front_right_walkers.append(walker)
                    self.front_right_distances.append(ego_distance)

            if len(self.front_right_distances) is not 0:
                front_min_index = np.argmin(self.front_right_distances)
                self.closest_front_right_walker = self.front_right_walkers[front_min_index]
                self.closest_distance_from_front_right_walker = self.front_right_distances[front_min_index]
            

            for walker in self.walkers_in_left_lane:
                if not walker.is_alive:
                    self.front_left_walkers = []
                    self.front_left_distances = []
                    continue
                walker_location = walker.get_location()

                ego_distance_front_left = walker_location.distance(self.front_location)
                ego_distance_rear_left  = walker_location.distance(self.rear_location)
                ego_distance            = walker_location.distance(self.ego_vehicle.get_location())

                if ego_distance_front_left < ego_distance_rear_left:
                    self.front_left_walkers.append(walker)
                    self.front_left_distances.append(ego_distance)

            if len(self.front_left_distances) is not 0:
                front_min_index = np.argmin(self.front_left_distances)
                self.closest_front_left_walker = self.front_left_walkers[front_min_index]
                self.closest_distance_from_front_left_walker = self.front_left_distances[front_min_index]


            for walker in self.walkers_in_same_lane:
                if not walker.is_alive:
                    self.front_walkers = []
                    self.front_distances = []
                    continue

                walker_location = walker.get_location()

                ego_distance_front_left = walker_location.distance(self.front_location)
                ego_distance_rear_left  = walker_location.distance(self.rear_location)
                ego_distance            = walker_location.distance(self.ego_vehicle.get_location())

                if ego_distance_front_left < ego_distance_rear_left:
                    self.front_walkers.append(walker)
                    self.front_distances.append(ego_distance)

            if len(self.front_distances) is not 0:
                front_min_index = np.argmin(self.front_distances)
                self.closest_front_walker = self.front_walkers[front_min_index]
                self.closest_distance_from_front_walker = self.front_distances[front_min_index]

