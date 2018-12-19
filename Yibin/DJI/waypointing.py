import numpy as np
import heapq
import cv2


def find_path(robot, env, destination):
	distance = np.full((80,50), 10000, dtype=float)
	heap = []
	paths = {}

	x, y = int(robot.x) // 10, int(robot.y) // 10

	# Set unreachable zones
	for obstacle in env.obstacles:
		l = round((obstacle.l - robot.width / 2) / 10)
		r = round((obstacle.r + robot.width / 2) / 10)
		b = round((obstacle.b - robot.length / 2) / 10)
		t = round((obstacle.t + robot.length / 2) / 10)
		distance[max(l,0):r + 1,max(b,0):t + 1] = np.inf

	distance[:round(robot.width / 2) // 10,:] = np.inf
	distance[-round(robot.width / 2) // 10:,:] = np.inf
	distance[:,:round(robot.length / 2) // 10] = np.inf
	distance[:,-round(robot.length / 2) // 10:] = np.inf

	# FOR VISUALIZATION OF NO-GO ZONES
	# dji_map = np.zeros((80,50,3))
	# for i in range(distance.shape[0]):
	#	for j in range(distance.shape[1]):
	#		if distance[i,j] > 10000:
	#			dji_map[i,j,:] = 255 # Color barriers white
	# cv2.imshow('path', cv2.resize(cv2.flip(dji_map.transpose((1,0,2)),0), (800,500)))
	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

	paths[(x, y)] = [[x, y]]
	directions = [[0,1],[1,0],[0,-1],[-1,0]]

	def update_heap(start_x, start_y):
		nonlocal heap
		for direction in directions:
			next_x, next_y = start_x + direction[0], start_y + direction[1]
			if distance[next_x, next_y] == 10000:
				heapq.heappush(heap, (distance[start_x, start_y], [start_x, start_y, next_x, next_y]))

	def update_paths(start_x, start_y, next_x, next_y):
		nonlocal distance, destination, paths, heap

		paths[(next_x, next_y)] = paths[(start_x, start_y)] + [[next_x, next_y]]
		if distance[next_x, next_y] <= distance[start_x,start_y] + 1:
			return

		distance[next_x,next_y] = distance[start_x,start_y] + 1

		if next_x == destination[0] // 10 and next_y == destination[1] // 10:
			return paths[(next_x, next_y)]

		update_heap(next_x, next_y)

	distance[x,y] = 1
	update_heap(x, y)
	count = 0
	while heap: 
		start_x, start_y, next_x, next_y = heapq.heappop(heap)[1]

		ret = update_paths(start_x, start_y, next_x, next_y)
		if ret:

			# FOR VISUALIZATION OF RETURNED PATH
			# try: # dji_map may or may not be already initialized earlier in code
			#	path = paths[(next_x, next_y)]
			#	for coord in path:
			#		dji_map[coord[0],coord[1],1] = 100 # Color the path green
			#	cv2.imshow('path', cv2.resize(cv2.flip(dji_map.transpose((1,0,2)),0), (800,500)))
			#	cv2.waitKey(0)
			#	cv2.destroyAllWindows()
			# except:
			#	dji_map = np.zeros((80,50,3))
			#	path = paths[(next_x, next_y)]
			#	for coord in path:
			#		dji_map[coord[0],coord[1],1] = 100
			#	cv2.imshow('path', cv2.resize(cv2.flip(dji_map.transpose((1,0,2)),0), (800,500)))
			#	cv2.waitKey(0)
			#	cv2.destroyAllWindows()

			return np.array(paths[(next_x, next_y)])*10
	print(destination, "Path not found")

def new_path(robot, path):
	if path is not None and len(path) > 0:
		destination = path[0]
		if (robot.x - destination[0])**2 + (robot.y - destination[1])**2 < 100:
			return path[1:]
		else:
			return path
			
def find_path_between_two_coordinates(robot, env, start, destination):
	"""path function that generates path between start point and destination.

	Args:
		param1 (Objects.Robot): a robot object.
		param2 (env): a gym environment.
		param3 (list or numpy.ndarray): start point.
		param4 (list or numpy.ndarray): end point. 

	Returns:
		numpy.ndarray: A numpy.ndarray of path.

	"""
	distance = np.full((80,50), 10000, dtype=float)
	heap = []
	paths = {}

	x, y = int(start[0]) // 10, int(start[1]) // 10

	# Set unreachable zones
	for obstacle in env.obstacles:
		l = round((obstacle.l - robot.width / 2) / 10)
		r = round((obstacle.r + robot.width / 2) / 10)
		b = round((obstacle.b - robot.length / 2) / 10)
		t = round((obstacle.t + robot.length / 2) / 10)
		distance[max(l,0):r + 1,max(b,0):t + 1] = np.inf

	distance[:round(robot.width / 2) // 10,:] = np.inf
	distance[-round(robot.width / 2) // 10:,:] = np.inf
	distance[:,:round(robot.length / 2) // 10] = np.inf
	distance[:,-round(robot.length / 2) // 10:] = np.inf

	paths[(x, y)] = [[x, y]]
	directions = [[0,1],[1,0],[0,-1],[-1,0]]

	def update_heap(start_x, start_y):
		nonlocal heap
		for direction in directions:
			next_x, next_y = start_x + direction[0], start_y + direction[1]
			if distance[next_x, next_y] == 10000:
				heapq.heappush(heap, (distance[start_x, start_y], [start_x, start_y, next_x, next_y]))

	def update_paths(start_x, start_y, next_x, next_y):
		nonlocal distance, destination, paths, heap

		paths[(next_x, next_y)] = paths[(start_x, start_y)] + [[next_x, next_y]]
		if distance[next_x, next_y] <= distance[start_x,start_y] + 1:
			return

		distance[next_x,next_y] = distance[start_x,start_y] + 1

		if next_x == destination[0] // 10 and next_y == destination[1] // 10:
			return paths[(next_x, next_y)]

		update_heap(next_x, next_y)

	distance[x,y] = 1
	update_heap(x, y)
	count = 0
	while heap: 
		start_x, start_y, next_x, next_y = heapq.heappop(heap)[1]

		ret = update_paths(start_x, start_y, next_x, next_y)
		if ret:
			return np.array(paths[(next_x, next_y)])*10
	print(destination, "Path not found")

def find_path_between_multiple_points(robot, env, coordinates_list):
	"""path function that generates path for multiple points.

	Args:
		param1 (Objects.Robot): a robot object.
		param2 (env): a gym environment.
		param3 (list or numpy arrays): a list of all coordinates. The first element and the last element must match. 

	Returns:
		list or numpy.ndarray: path to multiple points.
		

	"""
	assert len(coordinates_list) > 0, str(coordinates_list) + "Coordinates cannot be None"
	
	x, y = coordinates_list[0]
	total_path = find_path(env.robot, env, coordinates_list[0])
	print("Path to", coordinates_list)
	for i in range(len(coordinates_list) - 1):
		path = find_path_between_two_coordinates(robot, env, coordinates_list[i], coordinates_list[i+1])
		#print(total_path)
		total_path = np.append(total_path, path, axis=0)
	return total_path
	
def patrol(robot, env, cycle, coordinates_list):
	"""path function that generates path for robot walking around each coordinate.

	Args:
		param1 (Objects.Robot): a robot object.
		param2 (env): a gym environment.
		param3 (int): number of cycle of patrol
		param4 (list or numpy arrays): a list of all coordinates. The first element and the last element must match. 

	Returns:
		list or numpy.ndarray: path to finish the patrol.

	"""
	assert len(coordinates_list) > 0, str(coordinates_list) + "Coordinates cannot be None"
	assert coordinates_list[0] == coordinates_list[-1], "Start and end do not match"
	
	total_path = find_path_between_multiple_points(env.robot, env, coordinates_list)
	# path that excludes the part from current robot position to the first coordinate
	path_starting_at_first_coordinate = np.array([])
	for i in range(len(total_path)):
		if (total_path[i] == np.array(coordinates_list[0])).all():
			path_starting_at_first_coordinate = total_path[i:]
			break
	print("Patrol around", coordinates_list)
	for i in range(cycle - 1):
		total_path = np.append(total_path, path_starting_at_first_coordinate, axis=0)
	return total_path
	