#!/usr/bin/python3 
from vehicle import EgoVehicle
from sensor import Sensor, Lidar, CameraRGB, GNSS, IMU, ObstacleDetector, LaneInvasionDetector, Radar
from client import Client

def main():
    client = Client()                                       # create a new client 
    client.connect()                                        # connect the client 
    [blueprint, world, map]= client.get_simulation()        
    
    vehicle = EgoVehicle()                                  # create a new vehicle 
    vehicle.spawn(map)                                      # spawn the vehicle 
    vehicle.choose_model('model3', blueprint, world)
    vehicle_actor = vehicle.get_vehicle_actor()
    vehicle_transform = vehicle.get_vehicle_transform()
    print(vehicle_transform.location)
    print(vehicle_transform.rotation)

    lidar = Lidar()                  # create a sensor 
    lidar.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    lidar.set_rotation(0,0,0)
    lidar.set_location(0,0,0)
    lidar.set_simulation(blueprint, world, map)
    lidar.set_sensor()

    camera_rgb = CameraRGB()                 # create a sensor 
    camera_rgb.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    camera_rgb.set_rotation(0,0,0)
    camera_rgb.set_location(0,0,0)
    camera_rgb.set_simulation(blueprint, world, map)
    camera_rgb.set_sensor()

    gnss = GNSS()                 # create a sensor 
    gnss.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    gnss.set_rotation(0,0,0)
    gnss.set_location(0,0,0)
    gnss.set_simulation(blueprint, world, map)
    gnss.set_sensor()

    imu = IMU()                 # create a sensor 
    imu.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    imu.set_rotation(0,0,0)
    imu.set_location(0,0,0)
    imu.set_simulation(blueprint, world, map)
    imu.set_sensor()

    obs = ObstacleDetector()                 # create a sensor 
    obs.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    obs.set_rotation(0,0,0)
    obs.set_location(0,0,0)
    obs.set_simulation(blueprint, world, map)
    obs.set_sensor()

    lane = LaneInvasionDetector()                 # create a sensor 
    lane.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    lane.set_rotation(0,0,0)
    lane.set_location(0,0,0)
    lane.set_simulation(blueprint, world, map)
    lane.set_sensor()

    radar = Radar()                 # create a sensor 
    radar.set_vehicle(vehicle_actor)                        # give the vehicle that the sensor will be attached to  
    radar.set_rotation(0,0,0)
    radar.set_location(0,0,0)
    radar.set_simulation(blueprint, world, map)
    radar.set_sensor()

    radar.read()
    obs.read()
    imu.read()
    gnss.read()
    lidar.read()
    camera_rgb.read()

    vehicle.add_sensor(lidar)
    vehicle.add_sensor(camera_rgb)

    vehicle.wander()
    
    print('done.')

    
if __name__ == "__main__":
    main()