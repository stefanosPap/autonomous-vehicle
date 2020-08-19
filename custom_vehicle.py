
import glob
import os
import signal 
import sys
import time
import random 

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

#def signal_handler():
#    sys.exit(0)

 
def do_something(data):
    print(data)    
actor_list = []
try:
    client = carla.Client('localhost', 2000)    # create a client 
    client.set_timeout(2.0)                     # sets in seconds the maximum time a network call is allowed before blocking it 
                                                # default 5 seconds 
    world = client.get_world()                  # returns the world object currently active in the simulation 
    blueprint_library = world.get_blueprint_library() 
    
    
    map_ = world.get_map()
    
    # set possible points for vehicle creation 
    points = map_.get_spawn_points()
    spawn_point = random.choice(points) 
    
    # set vehicle 
    vehicle_bp = blueprint_library.filter('model3')[0] # get vehicle (tesla model3 from library)
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)
    actor_list.append(vehicle)
    
    # set lidar 
    blueprint_lidar = blueprint_library.find('sensor.lidar.ray_cast')
    blueprint_lidar.set_attribute('range', '30')
    blueprint_lidar.set_attribute('rotation_frequency', '10')
    blueprint_lidar.set_attribute('channels', '32')
    blueprint_lidar.set_attribute('lower_fov', '-30')
    blueprint_lidar.set_attribute('upper_fov', '30')
    blueprint_lidar.set_attribute('points_per_second', '56000')
    transform_lidar = carla.Transform(carla.Location(x=0.0, z=5.0))
    lidar = world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=vehicle)
    lidar.listen(lambda point_cloud: point_cloud.save_to_disk('/home/stefanos/Desktop/data/%.6d.ply' % point_cloud.frame))
    
    # set camera 
    cam_bp = None
    cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    cam_bp.set_attribute("image_size_x",str(1920))
    cam_bp.set_attribute("image_size_y",str(1080))
    cam_bp.set_attribute("fov",str(105))
    cam_location = carla.Location(2,0,1)
    cam_rotation = carla.Rotation(0,0,0)
    cam_transform = carla.Transform(cam_location,cam_rotation)
    ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    ego_cam.listen(lambda image: image.save_to_disk('/home/stefanos/Desktop/dataIm/%.6d.jpg' % image.frame))
    
    while True:    
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick()
        transform = vehicle.get_transform()
        transform.location.z += 2 
        spectator.set_transform(transform)
        world_snapshot = world.wait_for_tick()
    
    #while True:
    #   signal.signal(signal.SIGINT,signal_handler)
    
    # --------------
    # Game loop. Prevents the script from finishing.
    # --------------
        
        

     
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
