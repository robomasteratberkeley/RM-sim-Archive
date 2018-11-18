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
	# 	for j in range(distance.shape[1]):
	# 		if distance[i,j] > 10000:
	# 			dji_map[i,j,:] = 255 # Color barriers white
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
			# 	path = paths[(next_x, next_y)]
			# 	for coord in path:
			# 		dji_map[coord[0],coord[1],1] = 100 # Color the path green
			# 	cv2.imshow('path', cv2.resize(cv2.flip(dji_map.transpose((1,0,2)),0), (800,500)))
			# 	cv2.waitKey(0)
			# 	cv2.destroyAllWindows()
			# except:
			# 	dji_map = np.zeros((80,50,3))
			# 	path = paths[(next_x, next_y)]
			# 	for coord in path:
			# 		dji_map[coord[0],coord[1],1] = 100
			# 	cv2.imshow('path', cv2.resize(cv2.flip(dji_map.transpose((1,0,2)),0), (800,500)))
			# 	cv2.waitKey(0)
			# 	cv2.destroyAllWindows()

			return np.array(paths[(next_x, next_y)])*10
	print("Path not found")

def new_path(robot, path):
	if path is not None and len(path) > 0:
		destination = path[0]
		if (robot.x - destination[0])**2 + (robot.y - destination[1])**2 < 100:
			return path[1:]
		else:
			return path