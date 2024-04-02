#we need to develop an rrt algorithm where the grid is essentially the camera feed pixel coords

import numpy as np
import matplotlib.pyplot as plt

# Define the RRT class
class RRTStar:
    def __init__(self, start, goal, obstacles, grid_size, max_iter, step_size, min_dist):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.grid_size = grid_size
        self.max_iter = max_iter
        self.step_size = step_size
        self.min_dist = min_dist
        self.nodes = [start]
        self.parent = [0]
        self.cost = [0]

    def generate_random_point(self):
        return (np.random.randint(0, self.grid_size[0]), np.random.randint(0, self.grid_size[1]))

    def is_valid_point(self, point):
        if point[0] < 0 or point[0] >= self.grid_size[0] or point[1] < 0 or point[1] >= self.grid_size[1]:
            return False
        for obstacle in self.obstacles:
            if np.linalg.norm(np.array(point) - np.array(obstacle)) <= self.min_dist:
                return False
        return True

    def nearest_node(self, point):
        distances = [np.linalg.norm(np.array(point) - np.array(node)) for node in self.nodes]
        nearest_index = np.argmin(distances)
        return nearest_index

    def new_node(self, nearest_index, target):
        direction = np.array(target) - np.array(self.nodes[nearest_index])
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        new_point = tuple(np.array(self.nodes[nearest_index]) + direction.astype(int))
        return new_point

    def get_parent(self, new_node):
        distances = [np.linalg.norm(np.array(new_node) - np.array(node)) for node in self.nodes]
        min_dist_index = np.argmin(distances)
        return min_dist_index

    def cost_to_node(self, index):
        cost = 0
        while index != 0:
            cost += np.linalg.norm(np.array(self.nodes[index]) - np.array(self.nodes[self.parent[index]]))
            index = self.parent[index]
        return cost

    def rewire(self, new_node_index):
        for i in range(len(self.nodes)):
            if np.linalg.norm(np.array(self.nodes[i]) - np.array(self.nodes[new_node_index])) <= self.step_size:
                if self.cost[i] > self.cost_to_node(new_node_index) + np.linalg.norm(np.array(self.nodes[i]) - np.array(self.nodes[new_node_index])):
                    self.parent[i] = new_node_index
                    self.cost[i] = self.cost_to_node(new_node_index) + np.linalg.norm(np.array(self.nodes[i]) - np.array(self.nodes[new_node_index]))

    def plan_path(self):
        for _ in range(self.max_iter):
            target = self.generate_random_point()
            nearest_index = self.nearest_node(target)
            if self.is_valid_point(self.new_node(nearest_index, target)):
                new_node = self.new_node(nearest_index, target)
                self.nodes.append(new_node)
                self.parent.append(self.get_parent(new_node))
                self.cost.append(self.cost_to_node(self.parent[-1]) + np.linalg.norm(np.array(new_node) - np.array(self.nodes[self.parent[-1]])))
                self.rewire(len(self.nodes) - 1)

        # Extract the path
        path = []
        current = len(self.nodes) - 1
        while current != 0:
            path.append(self.nodes[current])
            current = self.parent[current]
        path.append(self.start)
        path.reverse()
        return path

def plot_pathfinding(start, goal, obstacles, grid_size, path):
    plt.figure(figsize=(8, 8))
    plt.plot([node[0] for node in path], [node[1] for node in path], '-o')
    plt.plot([start[0]], [start[1]], 'ro')
    plt.plot([goal[0]], [goal[1]], 'go')
    for obstacle in obstacles:
        plt.plot([obstacle[0]], [obstacle[1]], 'ko', markersize=10)
    plt.xlim(-1, grid_size[0])
    plt.ylim(-1, grid_size[1])
    plt.gca().invert_yaxis()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT* Path Planning')
    plt.grid(True)
    plt.show()
#Width x Height : (640,480) we can kinda make an assumption that the higher something is on the camera feed,
#  the further it is for our purposes
# Example of what the inputs might look like:
#       [100,200, person], [200, 400, sports ball], [200, 400, sports ball]
#       starting point would be bottom left corner of the camera  (0, 480)
#       Add controller logic for balls, if there is more than one ball, go to ball with smaller height/y (the closer one)
start = (0, 0)
goal = (9, 9)
obstacles = [(3, 3), (4, 4), (5, 5)]
grid_size = (10, 10)
max_iter = 1000
step_size = 1
min_dist = 1.5  # Minimum distance to avoid obstacles

# Create RRTStar instance and plan the path
rrt_star = RRTStar(start, goal, obstacles, grid_size, max_iter, step_size, min_dist)
path = rrt_star.plan_path()
plot_pathfinding(start = start, goal = goal, obstacles =obstacles, grid_size= grid_size, max_iter = max_iter, min_dist=min_dist)

