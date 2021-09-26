import carla
import numpy as np 

class ObstacleManager(object):
    
    def __init__(self, map, vehicle_actor, vehicle_list, world):
        self.map   = map
        self.world = world
        
        self.vehicle_actor  = vehicle_actor
        self.vehicle_list   = vehicle_list
        self.ego_vehicle    = self.vehicle_list[0]
        self.front_location = self.vehicle_actor.get_location() + carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        self.rear_location  = self.vehicle_actor.get_location() - carla.Location(self.vehicle_actor.bounding_box.extent.x, 0, 0)
        
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

        self.vehicles_in_lane        = []
        self.rear_right_vehicles     = []
        self.rear_right_distances    = []
        self.front_right_distances   = []
        self.vehicles_in_right_lane  = []
        self.vehicles_in_right_lane  = []
        self.vehicles_in_left_lane   = []
        self.front_general_vehicles  = []
        self.front_general_distances = []
        
    def check_general_closest_obstacles(self): 

        self.front_general_vehicles  = []
        self.front_general_distances = []

        for vehicle in self.vehicle_list[1:]:
            if not vehicle.is_alive:
                self.front_general_vehicles  = []
                self.front_general_distances = []
                continue

            vehicle_location   = vehicle.get_location()
            ego_distance_front = vehicle_location.distance(self.front_location)
            ego_distance_rear  = vehicle_location.distance(self.rear_location)
            ego_distance       = vehicle_location.distance(self.ego_vehicle.get_location())
            diff = abs(self.vehicle_actor.get_transform().rotation.yaw - vehicle.get_transform().rotation.yaw) 
            
            if ego_distance_front < ego_distance_rear and ego_distance < 35 and vehicle not in self.vehicles_in_lane:
                self.front_general_vehicles.append(vehicle)
                self.front_general_distances.append(ego_distance)

          
    def check_side_obstacles(self, waypoints, index, state):
    
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
                if not vehicle.is_alive:
                    self.vehicles_in_left_lane  = []
                    self.vehicles_in_right_lane = []
                    continue 
                ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                other_waypoint = self.map.get_waypoint(vehicle.get_location(), project_to_road=False, lane_type=carla.LaneType.Any)
                
                if other_waypoint == None:
        
                    continue
                
                # check if right lane exists 
                right_lane_id = None
                if state != "RIGHT":
                    if ego_waypoint is not None:
                        if ego_waypoint.get_right_lane() is not None:
                            right_lane_id = ego_waypoint.get_right_lane().lane_id

                # check if left lane exists 
                left_lane_id = None
                if state != "LEFT":
                    if ego_waypoint is not None:
                        if ego_waypoint.get_left_lane() is not None:
                            left_lane_id = ego_waypoint.get_left_lane().lane_id
         
                # check for side obstacles 
                if right_lane_id == other_waypoint.lane_id: 
                    self.vehicles_in_right_lane.append(vehicle)

                if left_lane_id == other_waypoint.lane_id:
                    self.vehicles_in_left_lane.append(vehicle)

            for vehicle in self.vehicles_in_right_lane:
                if not vehicle.is_alive:
                    self.front_right_vehicles   = []
                    self.front_right_distances  = []
                    self.rear_right_vehicles    = []
                    self.rear_right_distances   = []
                    continue    
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
                if self.front_right_distances[front_min_index] < 50:
                    self.closest_front_right_vehicle = self.front_right_vehicles[front_min_index]
                    self.closest_distance_from_front_right_vehicle = self.front_right_distances[front_min_index]
                else:
                    self.closest_front_right_vehicle = None
                    self.closest_distance_from_front_right_vehicle = float("inf")
            
            if len(self.rear_right_distances) is not 0:
                rear_min_index = np.argmin(self.rear_right_distances)
                if self.rear_right_distances[rear_min_index] < 50:
                    self.closest_rear_right_vehicle = self.rear_right_vehicles[rear_min_index]
                    self.closest_distance_from_rear_right_vehicle = self.rear_right_distances[rear_min_index]
                else:
                    self.closest_rear_right_vehicle = None
                    self.closest_distance_from_rear_right_vehicle = float("inf")

            for vehicle in self.vehicles_in_left_lane:
                if not vehicle.is_alive:
                    self.front_left_vehicles    = []
                    self.front_left_distances   = []
                    self.rear_left_vehicles     = []
                    self.rear_left_distances    = []
                    continue
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
                if self.front_left_distances[front_min_index] < 50:
                    self.closest_front_left_vehicle = self.front_left_vehicles[front_min_index]
                    self.closest_distance_from_front_left_vehicle = self.front_left_distances[front_min_index]
                else:
                    self.closest_front_left_vehicle = None
                    self.closest_distance_from_front_left_vehicle = float("inf")
                    
            if len(self.rear_left_distances) is not 0:
                rear_min_index = np.argmin(self.rear_left_distances)
                if self.rear_left_distances[rear_min_index] < 50:
                    self.closest_rear_left_vehicle = self.rear_left_vehicles[rear_min_index]
                    self.closest_distance_from_rear_left_vehicle = self.rear_left_distances[rear_min_index]
                else:
                    self.closest_rear_left_vehicle = None
                    self.closest_distance_from_rear_left_vehicle = float("inf")
  
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

        count = 0
        for vehicle in self.vehicle_list[1:]:
            if not vehicle.is_alive:
                count += 1
                self.vehicles_in_lane = []
                continue

            ego_waypoint   = self.map.get_waypoint(self.ego_vehicle.get_location(), project_to_road=False)
            other_waypoint = self.map.get_waypoint(vehicle.get_location(), project_to_road=False)
            
            if ego_waypoint == None or other_waypoint == None:
                continue
    
            diff = abs(self.vehicle_actor.get_transform().rotation.yaw - vehicle.get_transform().rotation.yaw) 
            
            if ego_waypoint.lane_id == other_waypoint.lane_id:
                self.vehicles_in_lane.append(vehicle)

        count = 0
        for vehicle in self.vehicles_in_lane:
            if not vehicle.is_alive:
                count += 1
                self.front_vehicles   = []
                self.front_distances  = []
                self.rear_vehicles    = []
                self.rear_distances   = []
                continue

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
            if self.front_distances[front_min_index] < 50:
                self.closest_front_vehicle = self.front_vehicles[front_min_index]
                self.closest_distance_from_front_vehicle = self.front_distances[front_min_index]
            else:
                self.closest_front_vehicle = None
                self.closest_distance_from_front_vehicle = float("inf")

        if len(self.rear_distances) is not 0:
            rear_min_index = np.argmin(self.rear_distances)
            if self.rear_distances[rear_min_index] < 50:
                self.closest_rear_vehicle = self.rear_vehicles[rear_min_index]
                self.closest_distance_from_rear_vehicle = self.rear_distances[rear_min_index]
            else:
                self.closest_rear_vehicle = None
                self.closest_distance_from_rear_vehicle = float("inf")
