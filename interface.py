import carla
from communicationMQTT import VehicleSubscriberStartStopMQTT, \
                              VehicleSubscriberCoorMQTT, \
                              VehicleSubscriberEnterMQTT, \
                              VehicleSubscriberDoneMQTT, \
                              VehiclePublisherMQTT, \
                              VehicleSubscriberForwardMQTT, \
                              VehicleSubscriberCoorForwardMQTT
class Interface(object):
    def __init__(self, world, map):
        self.world = world 
        self.map = map
        self.setupCom()

    def setupCom(self):
        self.pub_waypoint = VehiclePublisherMQTT(topic='waypoint_choose')
        self.pub = VehiclePublisherMQTT(topic='clean')
        self.pub_vel = VehiclePublisherMQTT(topic='speed_topic')
        self.pub_vel_conf = VehiclePublisherMQTT(topic='speed_configure')
        self.pub_forward = VehiclePublisherMQTT(topic='forward meters')

        self.sub_enter = VehicleSubscriberEnterMQTT(topic='enter')
        self.sub_done = VehicleSubscriberDoneMQTT(topic='done')
        self.sub_coor = VehicleSubscriberCoorMQTT(topic='coordinates')
        self.sub_coor_forward = VehicleSubscriberCoorForwardMQTT(topic='coordinates_forward')
        self.sub_forward = VehicleSubscriberForwardMQTT(topic='forward')

    def handle(self, start_waypoint):

        msg = {'value': ''}
        self.pub_waypoint.publish(msg)
            
        end_waypoints = [start_waypoint]
          
        text = {'value': ''}
        self.pub.publish(text)

        vel = {'velocity': 0}  
        self.pub_vel.publish(vel)
        self.pub_vel_conf.publish(vel)

        prev_point = []
        num = 1 
        
        while True:
            self.world.tick()
            if self.sub_enter.get_enter() == True:
                self.sub_enter.set_enter(False)
                self.pub.publish(text)
                coordinates = self.sub_coor.get_coordinates()
                
                if coordinates != []:
                    #point = [int(item) for item in coordinates.split()]
                    point = coordinates
                    if len(point) == 3 and point != prev_point: 
                        way = {'value': 'Location {} at: {}'.format(num, self.sub_coor.get_location())}
                        self.pub_waypoint.publish(way)
                        self.pub.publish({'value': ''})

                        num += 1  

                        prev_point = point 
                        point = carla.Location(point[0],point[1],point[2])
                        waypoint = self.map.get_waypoint(point, project_to_road=True, lane_type=carla.LaneType.Any)
                        end_waypoints.append(waypoint)
            

                break
        return end_waypoints 
    
    def handle_forward(self, start_waypoint):
        waypoints = []                
        waypoint = start_waypoint

        self.pub_forward.publish({'value': "Specify the distance in meters you want to go forward!"})
        while True:
            self.world.tick()
            if self.sub_enter.get_enter():
                self.sub_enter.set_enter(False)
                p = self.sub_coor_forward.get_coordinates()
                try:
                    p = int(p)

                except ValueError:
                    self.pub_waypoint.publish({'value': 'Invalid Value! Try again!'})
                    self.pub.publish({'value': ''})
                    continue
            
                self.pub.publish({'value': ''})
                self.pub_waypoint.publish({'value': 'Going forward {} meters'.format(p)})
                waypoint = start_waypoint
                
                while p > 0:
                    current_waypoints = waypoint.next_until_lane_end(1.0)
                    p = p - len(current_waypoints)
                    if p > 0:
                        waypoints = waypoints + current_waypoints
                    else:
                        waypoints = waypoints + current_waypoints[0:p]
                    waypoint = waypoints[len(waypoints) - 1].next(1.0)

                    if len(waypoint) != 1:
                        for i in range(len(waypoint)):
                            final_waypoint = waypoint[i].next_until_lane_end(1.0)
                            if abs(waypoint[i].transform.rotation.yaw - final_waypoint[len(final_waypoint) - 1].transform.rotation.yaw) < 3:
                                break 
                        waypoint = waypoint[i]

                    else:
                        waypoint = waypoint[0]   
                break
                
        return waypoints

    def handle_turn(self, start_waypoint, turn):
        
        waypoints = start_waypoint.next_until_lane_end(1.0)
        paths = waypoints[len(waypoints) - 1].next(1.0)
        print(len(paths))
        for i in range(len(paths)):
            
            ways = paths[i].next_until_lane_end(1.0)
            
            if  turn == "RIGHT":
                if (ways[len(ways) - 1].transform.rotation.yaw > waypoints[len(waypoints) - 1].transform.rotation.yaw):
                    self.pub_waypoint.publish({'value': 'Turn RIGHT at the next junction'})
                    self.pub.publish({'value': ''})
                    for i in range(len(ways)):
                        waypoints.append(ways[i])
                    print("RIGHT")
                    break
                else:
                    self.pub_waypoint.publish({'value': 'Unable to turn RIGHT at the next junction'})
                    self.pub.publish({'value': ''})
                    return []

            elif turn == "LEFT":    
                if ways[len(ways) - 1].transform.rotation.yaw < waypoints[len(waypoints) - 1].transform.rotation.yaw and turn == "LEFT":
                    self.pub_waypoint.publish({'value': 'Turn LEFT at the next junction'})
                    self.pub.publish({'value': ''})
                    for i in range(len(ways)):
                        waypoints.append(ways[i])
                    print("LEFT")
                    break 
                else:
                    self.pub_waypoint.publish({'value': 'Unable to turn LEFT at the next junction'})
                    self.pub.publish({'value': ''})
                    return []

            elif turn == "STRAIGHT":
                if abs(ways[len(ways) - 1].transform.rotation.yaw - waypoints[len(waypoints) - 1].transform.rotation.yaw) < 3:
                    self.pub_waypoint.publish({'value': 'Going STRAIGHT at the next junction'})
                    self.pub.publish({'value': ''})
                    for i in range(len(ways)):
                        waypoints.append(ways[i])
                    print("STRAIGHT")
                    break
                else:
                    self.pub_waypoint.publish({'value': 'Unable to go STRAIGHT at the next junction'})
                    self.pub.publish({'value': ''})
                    return []

        return waypoints