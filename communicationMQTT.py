import time
from commlib.msg import PubSubMessage, DataClass
from commlib.transports.mqtt import Subscriber, ConnectionParameters, Publisher

class VehiclePublisherMQTT(): 
    def __init__(self, topic):
        self.topic = topic  
        self.connection_parameters = ConnectionParameters(host='localhost', port=1883)
        self.publisher = Publisher(topic=self.topic, conn_params=self.connection_parameters)
    
    def publish(self, msg):
        self.publisher.publish(msg)
        
class VehicleSubscriberStartStopMQTT():
    def __init__(self, topic):
        self.start = False 
        self.stop = False
        self.topic = topic  
        self.subscribe()

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
    
    def subscribe(self):
        connection_parameters = ConnectionParameters(host='localhost', port=1883)
        sub = Subscriber(topic=self.topic, on_message=self.data_callback, conn_params=connection_parameters)
        sub.run()

class VehicleSubscriberVelocityMQTT():
    def __init__(self, topic):
        self.velocity = 0 
        self.topic = topic  
        self.subscribe()

    def data_callback(self, msg):
        if self.topic == 'speed_configure':
            self.velocity = msg['value']

    def get_velocity(self):
        return self.velocity 

    def subscribe(self):
        connection_parameters = ConnectionParameters(host='localhost', port=1883)
        sub = Subscriber(topic=self.topic, on_message=self.data_callback, conn_params=connection_parameters)
        sub.run()
