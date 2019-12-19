#!/usr/bin/python
import urllib.request
import requests
import time


class DroneIDNode:
    def __init__(self):
        self.url = "https://droneid.dk/rmuasd/get_list_csv.php"
        self.response = ""
        self.drone_list = []
        self.drone_info = {
                'id': 904,  # UAV ID, Group 1: 901, Group 2: 902, Group 3: 903, Group 4: 904
                'lat': 55.47176,  # Latitude [decimal degrees]
                'lon': 10.32402,  # Longitude [decimal degrees]
                'alt': 0.0,  # Altitude [m]
                'trk': 180,  # Optional: Intended path of travel, (compass course, True North ref) [degrees]
                'hsp': 10.0,  # Optional: Horizontal speed [m/s]
                'eng': 50,  # Optional: energy level (battery level) 0-100% 
                'sim': True,  # Simulation = True, real flight = False
                'rst': "Tissemand",  # Optional: Reported status, max 20 chars
                }
    
    def update_log(self, lat, lon, alt, trk=None, hsp=None, eng=None, sim=True, rst="Sim"):
        self.drone_info = {
                'id': 904,  # UAV ID, Group 1: 901, Group 2: 902, Group 3: 903, Group 4: 904
                'lat': lat,  # Latitude [decimal degrees]
                'lon': lon,  # Longitude [decimal degrees]
                'alt': alt,  # Altitude [m]
                'trk': trk,  # Optional: Intended path of travel, (compass course, True North ref) [degrees]
                'hsp': hsp,  # Optional: Horizontal speed [m/s]
                'eng': eng,  # Optional: energy level (battery level) 0-100%
                'sim': sim,  # Simulation = True, real flight = False
                'rst': rst,  # Optional: Reported status, max 20 chars
        }
    
    def send_log(self):
        msg = {'id': self.drone_info['id'], 'lat': self.drone_info['lat'], 'lon': self.drone_info['lon'],
               'alt': self.drone_info['alt'], 'trk': self.drone_info['trk'], 'hsp': self.drone_info['hsp'],
               'eng': self.drone_info['eng'], 'sim': self.drone_info['sim'], 'rst': self.drone_info['rst']}

        try:
            r = requests.post("https://droneid.dk/rmuasd/log.php", data=msg, timeout=5)
        except Exception as ex:
            template = "An exception of type {0} occurred. Arguments:\n{1!r}"
            message = template.format(type(ex).__name__, ex.args)
            print(message)
            return 404, "Failed"
        return r.status_code, r.reason

    def hack(self, id):
        msg = {'id': id, 'lat': self.drone_info['lat'], 'lon': self.drone_info['lon'],
               'alt': self.drone_info['alt'], 'trk': self.drone_info['trk'], 'hsp': self.drone_info['hsp'],
               'eng': self.drone_info['eng'], 'sim': self.drone_info['sim'], 'rst': self.drone_info['rst']}

        r = requests.post("https://droneid.dk/rmuasd/log.php", data=msg, timeout=2.0)
        return r.status_code, r.reason

    def update_drone_list(self):
        page = urllib.request.urlopen(self.url)
        self.response = page.read().decode().split("\n")
        self.drone_list = []
        for drone in self.response:
            if len(drone) != 0:
                self.drone_list.append(drone.split(","))

        epoch_time = int(time.time())
        for drone in self.drone_list:
            drone[1] = epoch_time - int(drone[1])
            drone[2] = float(drone[2])
            drone[3] = float(drone[3])
            drone[4] = float(drone[4])
            drone[5] = float(drone[5])
            drone[7] = float(drone[6])


if __name__ == "__main__":
    # Init class
    drone_node = DroneIDNode()

    a = [0, 0]
    # a = [55.4718040079795, 10.32307768478619]
    # Update drone info
    drone_node.update_log(lat=a[0], lon=a[1], alt=0.0, rst="Tissemand")

    # Send log
    # print(drone_node.hack(901))
    # print(drone_node.hack(902))
    print(drone_node.hack(903))
    # print(drone_node.hack(904))

    # Get drone list
    drone_node.update_drone_list()

    # Print our group
    info = ["NAME", "SECONDS", "LATITUDE", "LONGITUDE", "ALTITUDE", "TRACK", "SPEED", "SIMULATION", "STATUS"]
    print(info)
    for drones in drone_node.drone_list:
        print(drones)


