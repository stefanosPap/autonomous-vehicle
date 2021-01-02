import time
from commlib.msg import PubSubMessage, DataClass
from commlib.transports.mqtt import Subscriber, ConnectionParameters

class VehicleSubscriberMQTT():
    def __init__(self, topic):
        self.start = False 
        self.stop = False
        self.topic = topic  
        self.subscribe()

    def data_callback(self, msg):

        if self.topic == 'start_stop_topic':
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
        sub = Subscriber(topic=self.topic,
                         on_message=self.data_callback,
                         conn_params=connection_parameters)
        sub.run()
