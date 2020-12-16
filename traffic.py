import carla 

class Traffic(object):
    
    ###############
    # Constructor #
    ###############
    def __init__(self, world):
        self.traffic_state = "GREEN"
        self.world = world
        pass

    ######################################
    # Method for checking traffic lights #
    ######################################
    def check_traffic_lights(self, vehicle_actor):
        if vehicle_actor.is_at_traffic_light():
            traffic_light = vehicle_actor.get_traffic_light()
        
            if traffic_light != None:
                if traffic_light.get_state() == carla.TrafficLightState.Red:
                    print("Red")
                    self.traffic_state = "RED"

                elif traffic_light.get_state() == carla.TrafficLightState.Yellow:
                    print("Yellow")
                    self.traffic_state = "YELLOW"

                elif traffic_light.get_state() == carla.TrafficLightState.Green:
                    print("Green")
                    self.traffic_state = "GREEN"
    
        return self.traffic_state

    ######################################
    # Method for checking traffic lights #
    ######################################
    def check_signs(self, waypoint):
        landmarks = waypoint.get_landmarks(distance=3)
        landmark_names = []
        if len(landmarks) != 0:
            for j in range(len(landmarks)):
                self.world.debug.draw_box(carla.BoundingBox(landmarks[j].transform.location, carla.Vector3D(0.5,0.5,2)), landmarks[j].transform.rotation, 0.05, carla.Color(255,0,0,0),100)
                landmark_names.append(landmarks[j].name)
                print(landmarks[j].name)

        return landmarks