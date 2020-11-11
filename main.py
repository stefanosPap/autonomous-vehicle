from vehicle import EgoVehicle

def main():
    vehicle = EgoVehicle()
    vehicle.connect()
    vehicle.spawn()
    vehicle.choose_model('model3')
    vehicle.wander()
    
    print('done.')
    
    
if __name__ == "__main__":
    main()