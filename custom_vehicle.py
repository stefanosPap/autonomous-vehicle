
import glob
import os
import signal 
import sys
import time
import random 
import math 

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
    
    '''
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
    '''
    '''
    gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
    gnss_location = carla.Location(0,0,0)
    gnss_rotation = carla.Rotation(0,0,0)
    gnss_transform = carla.Transform(gnss_location,gnss_rotation)
    gnss_bp.set_attribute("sensor_tick",str(3.0))
    ego_gnss = world.spawn_actor(gnss_bp,gnss_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    def gnss_callback(gnss):
        print("GNSS measure:\n"+str(gnss)+'\n')
    ego_gnss.listen(lambda gnss: gnss_callback(gnss))
    '''
    
    # --------------
    # Add a Logarithmic Depth camera to ego vehicle. 
    # --------------
    '''    
    depth_cam = None
    depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
    depth_bp.set_attribute("image_size_x",str(1920))
    depth_bp.set_attribute("image_size_y",str(1080))
    depth_bp.set_attribute("fov",str(105))
    depth_location = carla.Location(2,0,1)
    depth_rotation = carla.Rotation(0,0,0)
    depth_transform = carla.Transform(depth_location,depth_rotation)
    depth_cam = world.spawn_actor(depth_bp,depth_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view
    depth_cam.listen(lambda image: image.save_to_disk('/home/stefanos/Desktop/dataLog/%.6d.jpg' % image.frame,carla.ColorConverter.LogarithmicDepth))
    '''    
    # --------------
    # Add a Depth camera to ego vehicle. 
    # --------------
    '''
    depth_cam02 = None
    depth_bp02 = world.get_blueprint_library().find('sensor.camera.depth')
    depth_bp02.set_attribute("image_size_x",str(1920))
    depth_bp02.set_attribute("image_size_y",str(1080))
    depth_bp02.set_attribute("fov",str(105))
    depth_location02 = carla.Location(2,0,1)
    depth_rotation02 = carla.Rotation(0,0,0)
    depth_transform02 = carla.Transform(depth_location02,depth_rotation02)
    depth_cam02 = world.spawn_actor(depth_bp02,depth_transform02,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view
    depth_cam02.listen(lambda image: image.save_to_disk('/home/stefanos/Desktop/dataDep/%.6d.jpg' % image.frame,carla.ColorConverter.Depth))
    '''
    # --------------
    # Add a new semantic segmentation camera to ego vehicle
    # --------------
    '''
    sem_cam = None
    sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    sem_bp.set_attribute("image_size_x",str(1920))
    sem_bp.set_attribute("image_size_y",str(1080))
    sem_bp.set_attribute("fov",str(105))
    sem_location = carla.Location(2,0,1)
    sem_rotation = carla.Rotation(0,0,0)
    sem_transform = carla.Transform(sem_location,sem_rotation)
    sem_cam = world.spawn_actor(sem_bp,sem_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    # This time, a color converter is applied to the image, to get the semantic segmentation view
    sem_cam.listen(lambda image: image.save_to_disk('/home/stefanos/Desktop/dataSeg/%.6d.jpg' % image.frame,carla.ColorConverter.CityScapesPalette))
    '''  

    # --------------
    # Add a new radar sensor to ego vehicle
    # --------------
    '''    
    rad_cam = None
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    rad_bp.set_attribute('horizontal_fov', str(35))
    rad_bp.set_attribute('vertical_fov', str(20))
    rad_bp.set_attribute('range', str(20))
    rad_location = carla.Location(x=2.8, z=1.0)
    rad_rotation = carla.Rotation(pitch=5)
    rad_transform = carla.Transform(rad_location,rad_rotation)
    rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    def rad_callback(radar_data):
        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)
                
            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))
    rad_ego.listen(lambda radar_data: rad_callback(radar_data))
    '''
    while True:    
        spectator = world.get_spectator()
        world_snapshot = world.wait_for_tick()
        transform = vehicle.get_transform()
        transform.location.z += 2 
        spectator.set_transform(transform)
        world_snapshot = world.wait_for_tick()
        
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')
