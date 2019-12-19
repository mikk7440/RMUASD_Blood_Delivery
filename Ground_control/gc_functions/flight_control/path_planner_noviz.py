import numpy as np
import math
import copy


flight_geometry_coords = [	[5547180, 1032514],[5547157, 1032445],[5547134, 1032488],[5547115, 1032430],[5547147, 1032418],
				[5547121, 1032341],[5547183, 1032280],[5547199, 1032329],[5547166, 1032335],[5547182, 1032384],
				[5547211, 1032356],[5547216, 1032375],[5547173, 1032419],[5547185, 1032456],[5547234, 1032407],
				[5547247, 1032452],[5547206, 1032466]]




class Waypoint:
	ID = 0
	parent = 0
	coords = []
	cost_to_reach = 99999999
	distance_to_start = 0
	distance_to_goal = 0
	done_checked = 0


######### Check to see if a line from a to b collides with any obstacle #################
from shapely.geometry import LineString
def check_line(a,b,obstacles):

	#swapped [0] and [1] cuz of lat = y and lon = x
	p1 = np.asarray((a[1],a[0]))
	p2 = np.asarray((b[1],b[0]))

	line_ok = False
	line1 = LineString([(p1[0], p1[1]), (p2[0], p2[1])])
	
	for a in obstacles:
		obs = copy.copy(a)
		obs.append(obs[0])
		for i in range(len(obs) - 1):
			if ((obs[i][0] > p1[0]) and (obs[i+1][0] > p1[0]) and (obs[i][0] > p2[0]) and (obs[i+1][0] > p2[0])) or ((obs[i][0] < p1[0]) and (obs[i+1][0] < p1[0]) and (obs[i][0] < p2[0]) and (obs[i+1][0] < p2[0])):
				line_ok = True
			elif ((obs[i][1] > p1[1]) and (obs[i+1][1] > p1[1]) and (obs[i][1] > p2[1]) and (obs[i+1][1] > p2[1])) or ((obs[i][1] < p1[1]) and (obs[i+1][1] < p1[1]) and (obs[i][1] < p2[1]) and (obs[i+1][1] < p2[1])):
				line_ok = True

			else:
				line2 = LineString([(obs[i][0],obs[i][1]), (obs[i+1][0],obs[i+1][1])])
				if line1.intersection(line2).is_empty:
					line_ok = True
				else:
					line_ok = False
					return line_ok
	return line_ok


######### Find the shortest route while avoiding obstacles ##############################
def find_route(start, goal, nodes, obstacles):	

	dijkstra_nodes = []

	id_counter = 0
	
	sa = Waypoint()
	sa.ID = id_counter
	sa.coords = start
	sa.cost_to_reach = 0
	sa.distance_to_start = 0
	sa.distance_to_goal = math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)	
	dijkstra_nodes.append(sa)

	id_counter = id_counter + 1

	for i in nodes:
		w = Waypoint()
		w.ID = id_counter
		w.coords = i
		w.distance_to_start = math.sqrt((i[0] - start[0])**2 + (i[1] - start[1])**2)
		w.distance_to_goal = math.sqrt((i[0] - goal[0])**2 + (i[1] - goal[1])**2)
		dijkstra_nodes.append(w)
		id_counter = id_counter + 1
	
	s = Waypoint()
	s.ID = id_counter
	s.coords = goal
	s.distance_to_start = math.sqrt((start[0] - goal[0])**2 + (start[1] - goal[1])**2)
	s.distance_to_goal = 0
	dijkstra_nodes.append(s)

	current_node = dijkstra_nodes[0]

	itterations = 0
	nodes_checked = 0

	while True:
		
		itterations = itterations + 1

		for i in dijkstra_nodes:
			nodes_checked = nodes_checked + 1
			if current_node != i and i.done_checked == 0:
				if check_line(current_node.coords, i.coords, obstacles) == True:
					ctr = current_node.cost_to_reach + math.sqrt((i.coords[0] - current_node.coords[0])**2 + (i.coords[1] - current_node.coords[1])**2)				
					if ctr < i.cost_to_reach:				
						i.cost_to_reach = ctr
						i.parent = current_node.ID 
			
		current_node.done_checked = 1

		lowest_cost_cost = 99999999
		lowest_cost_node = dijkstra_nodes[0] #[]
		for j in dijkstra_nodes:
			if j.cost_to_reach < lowest_cost_cost and j.done_checked == 0:
				lowest_cost_cost = j.cost_to_reach
				lowest_cost_node = j		
		
		current_node = lowest_cost_node

		if current_node == s:
			return_val = []
			while current_node != sa:
				current_node.coords[0]
				a = current_node.ID
				b = current_node.parent	
					
				return_val.append(current_node.coords)
				current_node = dijkstra_nodes[b]
					
			return return_val

		if current_node == sa:
			print("no solution")
			return []



def nogo_on_drone_check(li, obstacles):
	ret_val = True	
	for i in obstacles:
		circle_x = 0
		circle_y = 0
		circle_r = 0
		for a in i:
			circle_x = circle_x + a[1]
			circle_y = circle_y + a[0]

		circle_x = circle_x / len(i)
		circle_y = circle_y / len(i)

		for a in i:
			circle_r = circle_r + math.sqrt( (circle_x - a[1])**2 + (circle_y - a[0])**2 )
		circle_r = circle_r / len(i)
	

		li_to_center = math.sqrt( (circle_x - li[0])**2 + (circle_y - li[1])**2 )


		if li_to_center < circle_r:
			ret_val = False
	

	return ret_val


######### Check to see if a drone is in an obstacle ## returns waypoint if true #########
def nogo_on_drone(drone, nodes, obstacles, flight_geometry_coords):
	ret_val = False	
	for i in obstacles:
		circle_x = 0
		circle_y = 0
		circle_r = 0
		for a in i:
			circle_x = circle_x + a[1]
			circle_y = circle_y + a[0]

		circle_x = circle_x / (len(i) * 1.0)
		circle_y = circle_y / (len(i) * 1.0)

		for a in i:
			circle_r = circle_r + math.sqrt( (circle_x - a[1])**2 + (circle_y - a[0])**2 )
		circle_r = circle_r / (len(i) * 1.0)
	

		drone_to_center = math.sqrt( (circle_x - drone.x)**2 + (circle_y - drone.y)**2 )


		if drone_to_center < circle_r:

			closest_lighthouse = []
			closest_lighthouse_d = 99999999
			
			for li in nodes:
				drone_to_lighthouse = math.sqrt( (li[0] - drone.x)**2 + (li[1] - drone.y)**2 )
				if drone_to_lighthouse < closest_lighthouse_d:		#see if this new lighthouse is closer	
					#lighthouse_to_center = math.sqrt( (circle_x - li[0])**2 + (circle_y - li[1])**2 )
					#if lighthouse_to_center > circle_r:		#see if lighthouse is outside obstacle
					if nogo_on_drone_check(li, obstacles) == True:									
						if check_line( [drone.x,drone.y], li, [flight_geometry_coords] ) == True:		#check if the drone can fly to the lighthouse
								closest_lighthouse = li
								closest_lighthouse_d = drone_to_lighthouse

			ret_val = [closest_lighthouse]


	return ret_val







