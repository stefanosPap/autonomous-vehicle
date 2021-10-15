import carla 

class Traffic(object):
    """
    Description:
        Class Traffic is used to handle the state of traffic lights and signals 
    """  

    def __init__(self, world, map):
        """
        Description:
            Method __init__ is the Constructor of Class Traffic that initializes most of the used variables

        Args:
            world           (carla.World)           :    World object of CARLA API
            map             (carla.Map)             :    Map object of CARLA API
        """        

        self.traffic_state = None
        self.world = world
        self.lane_info = {}
        self.map = map


    def check_traffic_lights(self, vehicle_actor):
        """
        Description:
            Method check_traffic_lights is responsible for checking the state of traffic lights

        Args:
            vehicle_actor (carla.Vehicle) : The actor object of the autonomous vehicle

        Returns:
            str:  The state of the traffic light (GREEN, RED, YELLOW)
        """

        self.traffic_state = "GREEN"
        if vehicle_actor.is_at_traffic_light():
                
            traffic_light = vehicle_actor.get_traffic_light()
            #self.world.debug.draw_box(traffic_light.trigger_volume, traffic_light.get_transform().rotation, 0.05, carla.Color(255,0,0,0),100)
            if traffic_light != None:
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    self.traffic_state = "RED"

                elif traffic_light.get_state() == carla.TrafficLightState.Yellow:
                    self.traffic_state = "YELLOW"

                elif traffic_light.get_state() == carla.TrafficLightState.Green:
                    self.traffic_state = "GREEN"

        return self.traffic_state


    def check_signs(self, waypoint):
        """
        Description:
            Method check_signs is responsible for checking the closest traffic signs that exist

        Args:
            waypoint (carla.Waypoint): The current waypoint that is crossed by the vehicle 
        
        Returns:
            list: The list with the closer Landmarks 
        """        

        if isinstance(waypoint, carla.libcarla.Transform):
            loc = waypoint.location
            del waypoint
            waypoint = self.map.get_waypoint(loc)

        landmarks = waypoint.get_landmarks(distance=8)
        if len(landmarks) != 0:
            for j in range(len(landmarks)):
                self.world.debug.draw_box(carla.BoundingBox(landmarks[j].transform.location, carla.Vector3D(0.5,0.5,2)), landmarks[j].transform.rotation, 0.05, carla.Color(255,0,0,0),100)
                
        return landmarks
    

    def get_lane_info(self, waypoint):
        """
        Description:
            Method get_lane_info is responsible for getting the information of each lane of the road 

        Args:
            waypoint (carla.Waypoint): The current waypoint that is crossed by the vehicle 

        Returns:
            dictionary: A dictionary with the information of each lane 
        """

        print("Right lane type: ", waypoint.right_lane_marking.type)
        print("Left lane type: ", waypoint.left_lane_marking.type)
        print("Valid lane change: ", waypoint.lane_change)
        print("Lane type: ", waypoint.lane_change)

        self.right_lane_marking = waypoint.right_lane_marking.type
        self.left_lane_marking = waypoint.left_lane_marking.type
        self.lane_change = waypoint.lane_change
        self.lane_type = waypoint.lane_type

        self.lane_info['RightType'] = self.right_lane_marking
        self.lane_info['LeftType'] = self.left_lane_marking
        self.lane_info['LaneChange'] = self.lane_change
        self.lane_info['LaneType'] = self.lane_type

        return self.lane_info