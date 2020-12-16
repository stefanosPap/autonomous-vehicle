import carla 

def plot_axis(world, origin):
    length = 20

    world.debug.draw_line(origin.location,  origin.location + carla.Location(length, 0, 0), thickness=0.1, color=carla.Color(255,0,0), life_time=0)
    world.debug.draw_line(origin.location,  origin.location + carla.Location(0, length, 0), thickness=0.1, color=carla.Color(0,255,0), life_time=0)
    world.debug.draw_line(origin.location,  origin.location + carla.Location(0, 0, length), thickness=0.1, color=carla.Color(0,0,255), life_time=0)

    world.debug.draw_string(origin.location + carla.Location(length + 1, 0, 0), 'X', draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=100, persistent_lines=True)
    world.debug.draw_string(origin.location + carla.Location(0, length + 1, 0), 'Y', draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=100, persistent_lines=True)
    world.debug.draw_string(origin.location + carla.Location(0, 0, length + 1), 'Z', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=100, persistent_lines=True)


def check_traffic_lights(vehicle_actor):
    traffic_state = "GREEN"
    if vehicle_actor.is_at_traffic_light():
        traffic_light = vehicle_actor.get_traffic_light()
        
        if traffic_light != None:
            if traffic_light.get_state() == carla.TrafficLightState.Red:
                print("Red")
                traffic_state = "RED"

            elif traffic_light.get_state() == carla.TrafficLightState.Yellow:
                print("Yellow")
                traffic_state = "YELLOW"

            elif traffic_light.get_state() == carla.TrafficLightState.Green:
                print("Green")
                traffic_state = "GREEN"
    
    return traffic_state

def draw_vehicle_box(world, vehicle_actor, location, rotation, life_time):

    bb = vehicle_actor.bounding_box
    bbox = carla.BoundingBox(location, bb.extent)
    world.debug.draw_box(bbox, rotation, 0.1, carla.Color(255,0,0), life_time)

