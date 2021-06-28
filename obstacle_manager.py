import carla
import numpy as np 

class ObstacleManager(object):
    
    def __init__(self, map, vehicle_actor, vehicle_list, world):
        self.map   = map
        self.world = world
        
        self.vehicle_actor = vehicle_actor
        self.vehicle_list  = vehicle_list
        
        self.closest_front_vehicle      = None
        self.closest_rear_vehicle       = None
        self.closest_rear_left_vehicle  = None
        self.closest_rear_right_vehicle = None

        self.closest_distance_from_front_vehicle       = float('inf')
        self.closest_distance_from_front_right_vehicle = float('inf')
        self.closest_distance_from_rear_vehicle        = float('inf')
        self.closest_distance_from_rear_left_vehicle   = float('inf')
        self.closest_distance_from_rear_right_vehicle  = float('inf')

        self.closest_front_left_vehicle  = vehicle_list[0]
        self.closest_front_right_vehicle = vehicle_list[0]

        self.vehicles_in_lane       = []
        self.rear_right_vehicles    = []
        self.rear_right_distances   = []
        self.front_right_distances  = []
        self.vehicles_in_right_lane = []
        
    def check_side_obstacles(self, waypoints, index):

            self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
            self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
            self.ego_vehicle    = self.vehicle_list[0]

            self.vehicles_in_right_lane = []
            self.front_right_vehicles   = []
            self.front_right_distances  = []
            self.rear_right_vehicles    = []
            self.rear_right_distances   = []

            self.vehicles_in_left_lane  = []
            self.front_left_vehicles    = []
            self.front_left_distances   = []
            self.rear_left_vehicles     = []
            self.rear_left_distances    = []

            self.closest_distance_from_front_right_vehicle = float('inf')
            self.closest_distance_from_rear_right_vehicle  = float('inf')
            
            self.closest_distance_from_front_left_vehicle = float('inf')
            self.closest_distance_from_rear_left_vehicle  = float('inf')
                    
            self.closest_front_left_vehicle  = None
            self.closest_front_right_vehicle = None
            self.closest_rear_left_vehicle   = None
            self.closest_rear_right_vehicle  = None

            for vehicle in self.vehicle_list[1:]:

                ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                other_waypoint = self.map.get_waypoint(vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                
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

                # check for side obstacles 
                if right_lane_id == other_waypoint.lane_id: 
                    self.vehicles_in_right_lane.append(vehicle)

                if left_lane_id == other_waypoint.lane_id:
                    self.vehicles_in_left_lane.append(vehicle)

            for vehicle in self.vehicles_in_right_lane:
                vehicle_location = vehicle.get_location()

                ego_distance_front_right = vehicle_location.distance(self.front_location)
                ego_distance_rear_right  = vehicle_location.distance(self.rear_location)
                ego_distance             = vehicle_location.distance(self.ego_vehicle.get_location())

                if ego_distance_front_right < ego_distance_rear_right:
                    self.front_right_vehicles.append(vehicle)
                    self.front_right_distances.append(ego_distance)
                else:
                    self.rear_right_vehicles.append(vehicle)
                    self.rear_right_distances.append(ego_distance)

            if len(self.front_right_distances) is not 0:
                front_min_index = np.argmin(self.front_right_distances)
                self.closest_front_right_vehicle = self.front_right_vehicles[front_min_index]
                self.closest_distance_from_front_right_vehicle = self.front_right_distances[front_min_index]
            
            if len(self.rear_right_distances) is not 0:
                rear_min_index = np.argmin(self.rear_right_distances)
                self.closest_rear_right_vehicle = self.rear_right_vehicles[rear_min_index]
                self.closest_distance_from_rear_right_vehicle = self.rear_right_distances[rear_min_index]

            for vehicle in self.vehicles_in_left_lane:
                vehicle_location = vehicle.get_location()

                ego_distance_front_left = vehicle_location.distance(self.front_location)
                ego_distance_rear_left  = vehicle_location.distance(self.rear_location)
                ego_distance            = vehicle_location.distance(self.ego_vehicle.get_location())

                if ego_distance_front_left < ego_distance_rear_left:
                    self.front_left_vehicles.append(vehicle)
                    self.front_left_distances.append(ego_distance)
                else:
                    self.rear_left_vehicles.append(vehicle)
                    self.rear_left_distances.append(ego_distance)

            if len(self.front_left_distances) is not 0:
                front_min_index = np.argmin(self.front_left_distances)
                self.closest_front_left_vehicle = self.front_left_vehicles[front_min_index]
                self.closest_distance_from_front_left_vehicle = self.front_left_distances[front_min_index]

            if len(self.rear_left_distances) is not 0:
                rear_min_index = np.argmin(self.rear_left_distances)
                self.closest_rear_left_vehicle = self.rear_left_vehicles[rear_min_index]
                self.closest_distance_from_rear_left_vehicle = self.rear_left_distances[rear_min_index]

            
    def check_obstacles(self):

        self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.ego_vehicle    = self.vehicle_list[0]
        
        self.vehicles_in_lane = []
        self.front_vehicles   = []
        self.front_distances  = []
        self.rear_vehicles    = []
        self.rear_distances   = []

        self.closest_distance_from_front_vehicle = float('inf')
        self.closest_distance_from_rear_vehicle  = float('inf')

        self.closest_front_vehicle = None
        self.closest_rear_vehicle  = None
        
        for vehicle in self.vehicle_list[1:]:

            ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False)
            other_waypoint = self.map.get_waypoint(vehicle.get_location(), project_to_road=False)
            
            if ego_waypoint == None or other_waypoint == None:
                continue
            
            if ego_waypoint.lane_id == other_waypoint.lane_id:
                self.vehicles_in_lane.append(vehicle)
       
        for vehicle in self.vehicles_in_lane:
            vehicle_location = vehicle.get_location()

            ego_distance_front = vehicle_location.distance(self.front_location)
            ego_distance_rear  = vehicle_location.distance(self.rear_location)
            ego_distance       = vehicle_location.distance(self.ego_vehicle.get_location())

            if ego_distance_front < ego_distance_rear:
                self.front_vehicles.append(vehicle)
                self.front_distances.append(ego_distance)
            else:
                self.rear_vehicles.append(vehicle)
                self.rear_distances.append(ego_distance)

        if len(self.front_distances) is not 0:
            front_min_index = np.argmin(self.front_distances)
            self.closest_front_vehicle = self.front_vehicles[front_min_index]
            self.closest_distance_from_front_vehicle = self.front_distances[front_min_index]

        if len(self.rear_distances) is not 0:
            rear_min_index = np.argmin(self.rear_distances)
            self.closest_rear_vehicle = self.rear_vehicles[rear_min_index]
            self.closest_distance_from_rear_vehicle = self.rear_distances[rear_min_index]