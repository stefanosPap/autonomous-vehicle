import carla 
from sensor import Sensor, Lidar, CameraRGB, GNSS, IMU, ObstacleDetector, LaneInvasionDetector, Radar, CameraSemantic

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

def configure_sensor(vehicle_actor, vehicle_transform, blueprint, world, map, *args):
    sensors = {}
    for sensor in args:
        if sensor == "Lidar":
            lidar = Lidar()                                             # create a lidar sensor 
            lidar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
            lidar.set_rotation(0,0,0)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
            lidar.set_location(0,0,2)
            lidar.set_simulation(blueprint, world, map)
            lidar.set_sensor()
            lidar.read()
            sensors['lidar'] = lidar

        elif sensor == "Radar":
            radar = Radar()                                             # create a sensor 
            radar.set_vehicle(vehicle_actor)                            # give the vehicle that the sensor will be attached to  
            radar.set_rotation(pitch=5,yaw=0,roll=0)
            radar.set_location(X=2,Y=0,Z=1)
            radar.set_simulation(blueprint, world, map)
            radar.set_sensor(horizontal_fov=35, vertical_fov=20)
            radar.read()
            sensors['radar'] = radar            
        
        elif sensor == "LaneInvasion":
            lane = LaneInvasionDetector()                               # create a Lane Invasion detector sensor 
            lane.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
            lane.set_rotation(0,0,0)
            lane.set_location(0,0,0)
            lane.set_simulation(blueprint, world, map)
            lane.set_sensor()
            lane.read()
            sensors['lane'] = lane            
            
        elif sensor == "ObstacleDetector":
            obs = ObstacleDetector()                                    # create an Obstacle detector sensor 
            obs.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
            obs.set_rotation(vehicle_transform.rotation.pitch, vehicle_transform.rotation.yaw, vehicle_transform.rotation.roll)
            obs.set_location(X=2,Y=0,Z=1)
            obs.set_simulation(blueprint, world, map)
            obs.set_sensor(hit_radius=10)
            obs.read()
            sensors['obs'] = obs            

        elif sensor == "IMU":
            imu = IMU()                                                 # create an IMU sensor 
            imu.set_vehicle(vehicle_actor)                              # give the vehicle that the sensor will be attached to  
            imu.set_rotation(0,0,0)
            imu.set_location(0,0,0)
            imu.set_simulation(blueprint, world, map)
            imu.set_sensor()
            imu.read()
            sensors['imu'] = imu    

        elif sensor == "GNSS":
            gnss = GNSS()                                               # create a GNSS sensor 
            gnss.set_vehicle(vehicle_actor)                             # give the vehicle that the sensor will be attached to  
            gnss.set_rotation(0,0,0)
            gnss.set_location(0,0,0)
            gnss.set_simulation(blueprint, world, map)
            gnss.set_sensor()        
            gnss.read()
            sensors['gnss'] = gnss    

        elif sensor == "Camera Semantic":
            camera_sem = CameraSemantic()                               # create a Semantic Segmentation Camera sensor 
            camera_sem.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
            camera_sem.set_rotation(0,0,0)
            camera_sem.set_location(0,0,3)
            camera_sem.set_simulation(blueprint, world, map)
            camera_sem.set_sensor()
            camera_sem.read()
            sensors['camera_sem'] = camera_sem    

        elif sensor == "Camera RGB":  
            camera_rgb = CameraRGB()                                    # create an RGB Camera sensor 
            camera_rgb.set_vehicle(vehicle_actor)                       # give the vehicle that the sensor will be attached to  
            camera_rgb.set_rotation(0,0,0)
            camera_rgb.set_location(0,0,2)
            camera_rgb.set_simulation(blueprint, world, map)
            camera_rgb.set_sensor()
            camera_rgb.read()
            sensors['camera_rgb'] = camera_rgb    

    return sensors
