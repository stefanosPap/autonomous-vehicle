##################################################
# Script for generating other vehicles' movement #
##################################################
  
import carla 
from utilities import draw_vehicle_box
from vehicle import Vehicle
from client import Client

client = Client()                                       
client.connect()                                        # connect the client 
[blueprint, world, map]= client.get_simulation()
start_point = carla.Transform(carla.Location(x=-6.446170, y=-79.055023, z=0.275307), carla.Rotation(pitch=0.000000, yaw=92.004189, roll=0.000000))

for k in range(1,10,5):
        spawn_point = carla.Transform()
        spawn_point.location = start_point.location + carla.Location(k,38,0)
        spawn_point.rotation = start_point.rotation
        vehicle = Vehicle()                                  
        vehicle.choose_spawn_point(spawn_point)                 # spawn the vehicle 
        vehicle.choose_model('model3', blueprint, world)
        vehicle_actor = vehicle.get_vehicle_actor()
        for i in range(20):
            world.tick()
        vehicle_actor.set_autopilot(True) 
        draw_vehicle_box(world, vehicle_actor, spawn_point.location, spawn_point.rotation, 100)

while True:
    world.tick()

