#!/usr/bin/python3 
from vehicle import EgoVehicle
from sensor import Sensor
from sensor import Lidar
from client import Client

def main():
    client = Client()
    client.connect()
    [blueprint, world, map]= client.get_simulation()
    vehicle = EgoVehicle()
    vehicle.spawn(map)
    vehicle.choose_model('model3', blueprint, world)
    vehicle.wander()
    vehicle_actor = vehicle.get_vehicle_actor()
    sensor = Sensor('sensor.lidar.ray_cast')
    sensor.set_vehicle(vehicle)
    sensor.set_rotation(0,0,0)
    sensor.set_location(0,0,0)
    sensor.set_simulation(blueprint, world, map)
    vehicle.add_sensor(sensor)
    print('done.')
    
    
if __name__ == "__main__":
    main()