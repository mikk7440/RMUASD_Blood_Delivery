# ISSUES: Obstacles with few points should not be seen as circels





import numpy as np
import math
import copy
import time
import random



# import kml_open		# Athanas'
# import path_planner_noviz	# Path planner module
import gc_functions.flight_control.kml_open as kml_open
import gc_functions.flight_control.path_planner_noviz as path_planner_noviz


flight_geometry_coords = [	[5547180, 1032514],[5547157, 1032445],[5547134, 1032488],[5547115, 1032430],[5547147, 1032418],
                [5547121, 1032341],[5547183, 1032280],[5547199, 1032329],[5547166, 1032335],[5547182, 1032384],
                [5547211, 1032356],[5547216, 1032375],[5547173, 1032419],[5547185, 1032456],[5547234, 1032407],
                [5547247, 1032452],[5547206, 1032466]]


#test_obs = [[5547178.932, 1032452.3259999999], [5547182.593, 1032446.12], [5547179.068, 1032439.6739999999], [5547176.507999999, 1032441.442], [5547176.411, 1032450.388]]


target_x = 1032350	# Where we want the drone to go
target_y = 5547150



######### Find the min and max latitudes and longitudes #################################
min_lon = 99999999
max_lon = 0
min_lat = 99999999
max_lat = 0

for i in flight_geometry_coords:
    if( i[1] < min_lon ):
        min_lon = i[1]
    if( i[1] > max_lon ):
        max_lon = i[1]
    if( i[0] < min_lat ):
        min_lat = i[0]
    if( i[0] > max_lat ):
        max_lat = i[0]



# Drone object
class Drone:
    x = 0
    y = 0
    h = 0
    dx = 0
    dy = 0
    target_x = 0
    target_y = 0
    name = "drones"
    route = []



d0 = Drone()
d1 = Drone()
d2 = Drone()
d3 = Drone()


d0.x = 1032445.8475
d0.y = 5547179.185
d0.name = "0"

d1.x = 0  # 1032410
d1.y = 0  # 5547155
d1.name = "1"

d2.x = 0  # 1032444
d2.y = 0  # 5547174
d2.name = "2"

d3.x = 0  # 1032410
d3.y = 0  # 5547155
d3.name = "3"



drones = [d0, d1, d2, d3]




######### Make a zone and returns the list of Lighthouses ###############################
def make_zone(zone, border):
    return_val = []
    z = copy.copy(zone)
    z.append(zone[0])
    z.append(zone[1])
    for i in range(len(z) - 2):
        p1 = [0, 0]
        p1[0] = z[i + 0][1]
        p1[1] = z[i + 0][0]

        p2 = [0, 0]
        p2[0] = z[i + 1][1]
        p2[1] = z[i + 1][0]

        p3 = [0, 0]
        p3[0] = z[i + 2][1]
        p3[1] = z[i + 2][0]

        vector_1_2 = [p2[0] - p1[0], p2[1] - p1[1]]
        vector_3_2 = [p2[0] - p3[0], p2[1] - p3[1]]

        size_v_1_2 = math.sqrt(vector_1_2[0]**2 + vector_1_2[1]**2)
        size_v_3_2 = math.sqrt(vector_3_2[0]**2 + vector_3_2[1]**2)

        vector_1_2[0] = (vector_1_2[0] * 1.0) / (size_v_1_2)
        vector_1_2[1] = (vector_1_2[1] * 1.0) / (size_v_1_2)
        vector_3_2[0] = (vector_3_2[0] * 1.0) / (size_v_3_2)
        vector_3_2[1] = (vector_3_2[1] * 1.0) / (size_v_3_2)

        size_v_1_2 = math.sqrt(vector_1_2[0]**2 + vector_1_2[1]**2)
        size_v_3_2 = math.sqrt(vector_3_2[0]**2 + vector_3_2[1]**2)

        avg_vector = [(vector_1_2[0] + vector_3_2[0]), (vector_1_2[1] + vector_3_2[1])]
        size_v_avg = math.sqrt(avg_vector[0]**2 + avg_vector[1]**2)

        avg_vector[0] = border * avg_vector[0] / size_v_avg
        avg_vector[1] = border * avg_vector[1] / size_v_avg

        return_val.append([p2[0] + avg_vector[0], p2[1] + avg_vector[1]])

    return return_val


######### drone_to_obstacle #############################################################
def drone_to_obstacle(drone, margin):
    return_val = []
    #return_val.append([drone.y + (drone.dy * margin), drone.x + (drone.dx * margin)])
    #return_val.append([drone.y + (drone.dx * margin), drone.x - (drone.dy * margin)])
    #return_val.append([drone.y - (drone.dy * margin), drone.x - (drone.dx * margin)])
    #return_val.append([drone.y - (drone.dx * margin), drone.x + (drone.dy * margin)])
    return_val.append([drone.y + margin, drone.x + margin])
    return_val.append([drone.y + margin, drone.x - margin])
    return_val.append([drone.y - margin, drone.x - margin])
    return_val.append([drone.y - margin, drone.x + margin])
    return return_val


######### See if any drones are too close ###############################################
def danger_close(drone, drones, d):
    ret_val = []

    drone_x_now = drone.x
    drone_y_now = drone.y
    drone_x_future = drone.x + drone.dx
    drone_y_future = drone.y + drone.dy

    for i in drones:
        d1 = math.sqrt( (drone_x_now - i.x)**2 + (drone_y_now - i.y)**2 )
        d2 = math.sqrt( (drone_x_now - (i.x + i.dx))**2 + (drone_y_now - (i.y + i.dy))**2 )
        d3 = math.sqrt( (drone_x_future - i.x)**2 + (drone_y_future - i.y)**2 )
        d4 = math.sqrt( (drone_x_future - (i.x + i.dx))**2 + (drone_y_future - (i.y + i.dy))**2 )

        if (d1 < d or d2 < d or d3 < d or d4 < d) and drone.name != i.name:
            ret_val.append(i)

    return ret_val





obstacles_new = []
obstacles_old = []
obstacles = []

static_lighthouses = []
static_lighthouses = static_lighthouses + make_zone(flight_geometry_coords, 3)# + make_zone(test_obs, 3)

static_obstacles = []
static_obstacles.append(flight_geometry_coords)
#static_obstacles.append(test_obs)
was_inside_obstacle = False

new_obstacles = False
drone_danger = False
obstacle_danger = False
stuck_in_big = False

path = []

obstacle_sample_time = 9999999999
obstacle_sample_rate = 0.5			

target_old = []


path_itt = 0
counter_obst = 0



def plan_path(goal_x, goal_y):

    global path
    global path_itt

    global obstacles_new
    global obstacles_old
    global obstacles
    global target_old
    global was_inside_obstacle
    global counter_obst
    global stuck_in_big

    status = []

    obstacles = []
    obstacles_new = []

    lighthouses = static_lighthouses
    obstacles = copy.copy(static_obstacles)


    # skal ikke kore super ofte
    obstacles_new = kml_open.getlatlon()

    #obstacles_new = []


    # see if there are new obstacles
    if obstacles_new != obstacles_old:
        new_obstacles = True
        status.append("new obstacles")
        obstacles_old = obstacles_new
    else:
        new_obstacles = False


    # see if the drone is close to other drones
    risky_drones = danger_close(drones[0], drones, 20)
    if len(risky_drones) > 0:
        drone_danger = True
        status.append("drone danger")
        # print("Drone danger")
        for rd in risky_drones:
            drone_obs = drone_to_obstacle(rd, 7)
            obstacles.append(drone_obs)
            lighthouses = lighthouses + make_zone(drone_obs, 3)

    else:
        drone_danger = False

    bigger_zones = []
    for i in obstacles_new:
        if i != []:
            bigger_zones.append(make_zone( make_zone(i, 2), 2))

    #make lighthouses from the new obstacles
    for i in bigger_zones:
        # if i != []:
        lighthouses = lighthouses + make_zone(i, 4)

    #add the obstacles to the list of obstacles
    for i in bigger_zones:
        # if i != []:
        obstacles.append(i)



    #see if the drone is in an obstacle
    if counter_obst == 10 or stuck_in_big:
        path_1 = path_planner_noviz.nogo_on_drone(drones[0], lighthouses, bigger_zones, flight_geometry_coords)
        if path_1 != False:
            stuck_in_big = True
        else:
            stuck_in_big = False
        counter_obst = 0
    else:
        path_1 = path_planner_noviz.nogo_on_drone(drones[0], lighthouses, obstacles_new, flight_geometry_coords)
        counter_obst += 1

    if path_1 != False:
        # print( "Drone is in obstacle")
        status.append("inside object")
        path_itt = 0
        obstacle_danger = True
        path = path_1
        target_old = path
        was_inside_obstacle = True

    elif (drone_danger == True) or (new_obstacles == True) or (target_old != [goal_x, goal_y]) or was_inside_obstacle:
        # print("Calculate new path" )
        was_inside_obstacle = False
        path_itt = 0
        target_old = [goal_x, goal_y]
        path = path_planner_noviz.find_route([drones[0].x, drones[0].y], [goal_x, goal_y], lighthouses, obstacles)
        path.reverse()

    else:
        was_inside_obstacle = False
        # print("Fly")
        path = path

    return path, status



def set_drone_pos(index, x, y):

    drones[index].dx = x - drones[index].x
    drones[index].dy = y - drones[index].y

    drones[index].x = x
    drones[index].y = y





# while True:
#
#     print ""
#
#
#     print "Target: " + str(target_x) + ", " + str(target_y)
#
#     #set_drone_pos(drones, 0, x, y)
#     set_drone_pos(drones, 1, 0, 0)
#     set_drone_pos(drones, 2, 0, 0)
#     set_drone_pos(drones, 3, 0, 0)
#
#     path = plan_path(target_x, target_y)
#
#     drones[0].target_x = path[path_itt][0]
#     drones[0].target_y = path[path_itt][1]
#
#     print "path: " + str(path)
#
#     #we dont need this in the simulation/irl
#     if math.sqrt((drones[0].target_x - drones[0].x)**2 + (drones[0].target_y - drones[0].y)**2) < 1:
#         print "At waypoint"
#         if len(path) > 1:
#             path_itt = path_itt + 1
#
#         #if len(path) == path_itt or len(path) == 1:
#     if 1 == 1:
#         target_x = random.randint(min_lon, max_lon)
#         target_y = random.randint(min_lat, max_lat)
#         path_itt = 0

