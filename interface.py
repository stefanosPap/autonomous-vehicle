import carla
import numpy as np
from utilities import plot_axis
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
        #w = start_waypoint.previous(1.0)
        #w = w[0]
        #self.world.debug.draw_string(w.transform.location, 'F', draw_shadow=False, color=carla.Color(r=255, g=0, b=255), life_time=1000, persistent_lines=True)
        waypoints = [start_waypoint]
        #if not start_waypoint.is_junction:
        try:
            waypoints = start_waypoint.next_until_lane_end(1.0)
        except RuntimeError:
            pass 
        
        #waypoint = start_waypoint
        
        #while True:
        #    next = waypoint.next(1.0)
        #    for i in range(len(next)):
        #        if next[i].lane_id == waypoint.lane_id:
        #            waypoints.append(next[i])
        #            break
        #    if next[i].is_junction:
        #        break
        #    waypoint = next[i]
        
        #while True:
        #    if waypoints[len(waypoints) - 1].next(1.0)[0].is_junction:
        #        break
        #    waypoints += waypoints[len(waypoints) - 1].next_until_lane_end(1.0)
        
        #print(waypoints[len(waypoints) - 1].is_junction)
        #print(waypoints[0].is_junction)
        paths = waypoints[len(waypoints) - 1].next(1.0)
        for i in range(len(paths)):
            
            ways = paths[i].next_until_lane_end(1.0)
            #print(ways[len(ways) - 1].transform.rotation.yaw)
            #print(ways[0].transform.rotation.yaw)
            vec1 = ways[0].transform.get_forward_vector()
            vec1 = [vec1.x, vec1.y]
            
            vec2 = ways[len(ways) - 1].transform.get_forward_vector()
            vec2 = [vec2.x, vec2.y]
            
            unit_vector_1 = vec1 / np.linalg.norm(vec1)
            unit_vector_2 = vec2 / np.linalg.norm(vec2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arctan(dot_product)
            
            if round(angle, 1) == 0.8 and turn == "STRAIGHT": 
                self.pub_waypoint.publish({'value': 'Going STRAIGHT at the next junction'})
                self.pub.publish({'value': ''})
                for i in range(len(ways)):
                    waypoints.append(ways[i])
                break
            
            elif (ways[len(ways) - 1].transform.rotation.yaw - ways[0].transform.rotation.yaw) > 10 and turn == "RIGHT":
                self.pub_waypoint.publish({'value': 'Turn RIGHT at the next junction'})
                self.pub.publish({'value': ''})
                for i in range(len(ways)):
                    waypoints.append(ways[i])
                break
            
            elif ways[len(ways) - 1].transform.rotation.yaw - ways[0].transform.rotation.yaw < 0 and turn == "LEFT":
                self.pub_waypoint.publish({'value': 'Turn LEFT at the next junction'})
                self.pub.publish({'value': ''})
                for i in range(len(ways)):
                    waypoints.append(ways[i])
                break 
            

            
            elif i == len(paths) - 1:
                self.pub_waypoint.publish({'value': 'Unable to go {} at the next junction'.format(turn)})
                self.pub.publish({'value': ''})
                return []

        self.world.debug.draw_string(waypoints[len(waypoints) - 1].transform.location, 'X', draw_shadow=False, color=carla.Color(r=0, g=0, b=255), life_time=1000, persistent_lines=True)
        return waypoints
    
    def turn_info(self, start_waypoint):
        
        # This try except block is used in case of the start waypoint is at the end of the lane and it cannot produse new waypoints. 
        # Therefore it throws a RuntimeError and we use as waypoints the starting waypoint  
        waypoints = [start_waypoint]
        try:
            waypoints = start_waypoint.next_until_lane_end(1.0)
        except RuntimeError:
            pass 
            
        paths = waypoints[len(waypoints) - 1].next(1.0)
        turn = "Possible paths:"
        for i in range(len(paths)):

            ways = paths[i].next_until_lane_end(1.0)

            #self.world.debug.draw_arrow(ways[0].transform.location, carla.Location(x=vec.x, y=vec.y, z=vec.z), thickness=0.1, color=carla.Color(255,0,0), life_time=0)
            #loc1 = carla.Location(x=vec1.x, y=vec1.y, z=vec1.z)
            #self.world.debug.draw_point(loc, size=0.1, color=carla.Color(255,0,0), life_time=0)
            
            vec1 = ways[0].transform.get_forward_vector()
            vec1 = [vec1.x, vec1.y]
            
            vec2 = ways[len(ways) - 1].transform.get_forward_vector()
            vec2 = [vec2.x, vec2.y]
            
            unit_vector_1 = vec1 / np.linalg.norm(vec1)
            unit_vector_2 = vec2 / np.linalg.norm(vec2)
            dot_product = np.dot(unit_vector_1, unit_vector_2)
            angle = np.arctan(dot_product)
            print("------------------------")
            print(ways[0].transform.rotation.yaw)
            print(ways[len(ways) - 1].transform.rotation.yaw)
            print(angle)
            print("------------------------")
            '''
            if angle > 10:
                turn += " RIGHT"
                            
            elif ways[len(ways) - 1].transform.rotation.yaw < ways[0].transform.rotation.yaw:
                turn += " LEFT"
            
            elif angle < 3: 
                turn += " STRAIGHT"
            '''

            if round(angle, 1) == 0.8: 
                turn += " STRAIGHT"            
           
            elif (ways[len(ways) - 1].transform.rotation.yaw - ways[0].transform.rotation.yaw) > 10:
                turn += " RIGHT"
                            
            elif ways[len(ways) - 1].transform.rotation.yaw - ways[0].transform.rotation.yaw < 0 :
                turn += " LEFT"

            

            #self.world.debug.draw_arrow(ways[len(ways) - 1].transform.location, carla.Location(x=vec.x, y=vec.y, z=vec.z), thickness=0.1, color=carla.Color(0,0,0), life_time=0)
            #self.world.debug.draw_point(loc, size=0.1, color=carla.Color(255,0,0), life_time=0)

            #self.world.debug.draw_string(ways[0].transform.location, '{}'.format(round(ways[0].transform.rotation.yaw)), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=1000, persistent_lines=True)
            for j in range(1, len(ways)):
                self.world.debug.draw_string(ways[j].transform.location, '{}'.format(round(ways[j].transform.rotation.yaw)), draw_shadow=False, color=carla.Color(r=0, g=0, b=0), life_time=1000, persistent_lines=True)
        
        self.pub_waypoint.publish({'value': turn})