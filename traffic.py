import carla 

class Traffic(object):
    
    ###############
    # Constructor #
    ###############
    def __init__(self, world, map):
        self.traffic_state = None
        self.world = world
        self.lane_info = {}
        self.map = map

    ######################################
    # Method for checking traffic lights #
    ######################################
    def check_traffic_lights(self, vehicle_actor, waypoint):
        
        if vehicle_actor.is_at_traffic_light():
        
            traffic_light = vehicle_actor.get_traffic_light()
        
            if traffic_light != None:
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    self.traffic_state = "RED"

                elif traffic_light.get_state() == carla.TrafficLightState.Yellow:
                    self.traffic_state = "YELLOW"

                elif traffic_light.get_state() == carla.TrafficLightState.Green:
                    self.traffic_state = "GREEN"

        return self.traffic_state

    ######################################
    # Method for checking traffic lights #
    ######################################
    def check_signs(self, waypoint):
        
        if isinstance(waypoint, carla.libcarla.Transform):
            loc = waypoint.location
            del waypoint
            waypoint = self.map.get_waypoint(loc)

        landmarks = waypoint.get_landmarks(distance=5)
        if len(landmarks) != 0:
            for j in range(len(landmarks)):
                self.world.debug.draw_box(carla.BoundingBox(landmarks[j].transform.location, carla.Vector3D(0.5,0.5,2)), landmarks[j].transform.rotation, 0.05, carla.Color(255,0,0,0),100)
                #print(landmarks[j].name, landmarks[j].type, landmarks[j].value, landmarks[j].orientation)

        return landmarks
    
    ###############################
    # Method for taking lane info #
    ###############################
    def get_lane_info(self, waypoint):
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


