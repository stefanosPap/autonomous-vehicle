import time
import carla 
from commlib.msg import PubSubMessage, DataClass
from commlib.transports.mqtt import Subscriber, ConnectionParameters, Publisher

class VehiclePublisherMQTT(): 
    def __init__(self, topic):
        self.topic = topic  
        self.connection_parameters = ConnectionParameters(host='localhost', port=1883)
        self.publisher = Publisher(topic=self.topic, conn_params=self.connection_parameters)
    
    def publish(self, msg):
        self.publisher.publish(msg)

class VehicleSubscriberMQTT():
    def __init__(self, topic):  
        self.topic = topic 
        self.subscribe()

    def subscribe(self):
        connection_parameters = ConnectionParameters(host='localhost', port=1883)
        sub = Subscriber(topic=self.topic, on_message=self.data_callback, conn_params=connection_parameters)
        sub.run()

class VehicleSubscriberPositionMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):  
        super().__init__(topic)
        self.pos = None 

    def data_callback(self, msg):
        if self.topic == 'position':
            self.pos = msg['position']
    
    def get_pos(self):
        return self.pos 

class VehicleSubscriberStartStopMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.start = False 
        self.stop = False

    def data_callback(self, msg):
        if msg['trigger'] == "start":
            self.start = True
            self.stop = False  
        if msg['trigger'] == "stop":
            self.stop = True
            self.start = False 

    def get_start(self):
        return self.start 

    def get_stop(self):
        return self.stop

    def set_start(self, start):
        self.start = start 

    def set_stop(self, stop):
        self.stop = stop 
    
class VehicleSubscriberVelocityMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.velocity = 0 

    def data_callback(self, msg):
        if self.topic == 'speed_configure':
            self.velocity = msg['velocity']

    def get_velocity(self):
        return self.velocity 


class VehicleSubscriberLeftRightMQTT(VehicleSubscriberMQTT):
    def __init__(self, topic):
        super().__init__(topic)
        self.turn = None  

    def data_callback(self, msg):
        if self.topic == 'turn':
            self.turn = msg['value']

    def get_turn(self):
        return self.turn 
    
    def set_turn(self, value):
        self.turn = value 
