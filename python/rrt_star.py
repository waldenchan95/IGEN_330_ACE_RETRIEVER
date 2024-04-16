import pygame
import sys
import math
import numpy as np

# Path List
pathList = []
# Latest Path
latestPath = []
# Cost List
costList = []
# nodes of interest 
nodesOfInterest = []

#Best Path
bestPath = []

betterNodeX = 0
betterNodeY = 0

betterNodes = []

# Reconfigure Nodes List
reconfigureNodes = []
reconfigureNodesCost = []

# Exploration Parameters
std_dev = 300  # Random Node Generation Standard Deviation Parameter
max_growth_radius = 30  # Largest Length a new branch can grow
search_radius = 75 # radius of nodes included when rewiring branch

goal_radius = 100
# Visualisation Parameters
best_path_index = 0 # color this path green

# -------------------- FUNCTIONS SECTION -------------------- #

# -- Path Generation -- #

# Generate Random Node that is biased towards being generated near the goal
def generate_random_node(std_dev):
    
    x_bias = int(np.random.normal(endNode[0], std_dev))
    y_bias = int(np.random.normal(endNode[1], std_dev))
    
    #generates new x value if x value is out of bounds
    while( x_bias < 0 or x_bias > WIDTH):
        x_bias = int(np.random.normal(endNode[0], std_dev))
    #generates new y value if y value is out of bounds
    while( y_bias < 0 or y_bias > HEIGHT): 
        y_bias = int(np.random.normal(endNode[1], std_dev))

    random_node = np.array([x_bias,y_bias])

    return random_node

# normalize distance between nodes than multiply by max spread radius
def normalize_by_radius(random_node, nearest_node, radius):
    # Calculate the vector between new_node and nearest_node
    vector = random_node - nearest_node

    if np.linalg.norm(vector) > radius:
        # Normalize the vector to obtain the unit vector
        unit_vector = vector / np.linalg.norm(vector)
        # Scale the unit vector by the radius
        scaled_vector = radius * unit_vector
        # Calculate the new node position
        result_node = nearest_node + scaled_vector
        return result_node
    else:
        return random_node 

# find nearest node
# find list index containing nearest node
# find path index containing neartest node
# find nearest node
def find_nearest_node(random_node):
    min_distance = float('inf')
    list_index = 0
    path_index = 0
    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(random_node - node)
            if distance < min_distance:
                min_distance = distance
                list_index = x
                path_index = y
    return pathList[list_index][path_index]

def find_nearest_node_list_index(random_node):
    min_distance = float('inf')
    list_index = 0
    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(random_node - node)
            if distance < min_distance:
                min_distance = distance
                list_index = x
    return list_index

def find_nearest_node_path_index(random_node):
    min_distance = float('inf')
    path_index = 0
    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(random_node - node)
            if distance < min_distance:
                min_distance = distance
                path_index = y
    return path_index

def calculate_node_cost(list_index, path_index, new_node):
    path = pathList[list_index]
    cost = 0

    final_distance = np.linalg.norm(new_node-path[(len(path)-1)])
    
    for x in range(path_index - 1):
        cost += np.linalg.norm(path[x + 1] - path[x])
    
    cost += final_distance

    return cost

def calculate_cost(list_index, path_index):
    path = pathList[list_index]
    cost = 0
    
    for x in range(path_index):
        cost += np.linalg.norm(path[x + 1] - path[x])

    return cost

def find_lowest_cost_list_index(normalized_node, search_radius):
    
    smallest_cost = float('inf')
    smallest_cost_list_index = 0
    smallest_cost_path_index = 0

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(normalized_node - node)
            if distance <= search_radius:
                cost = calculate_node_cost(x, y, normalized_node)
                if cost < smallest_cost:
                    smallest_cost = cost
                    smallest_cost_list_index = x
                    smallest_cost_path_index = y
    
    return smallest_cost_list_index

def find_lowest_cost_path_index(normalized_node, search_radius):
    
    smallest_cost = float('inf')
    smallest_cost_list_index = 0
    smallest_cost_path_index = 0

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(normalized_node - node)
            if distance <= search_radius:
                cost = calculate_node_cost(x, y, normalized_node)
                if cost < smallest_cost:
                    smallest_cost = cost
                    smallest_cost_list_index = x
                    smallest_cost_path_index = y
    
    return smallest_cost_path_index

def find_reconfigurable_nodes(test_node):

    global reconfigureNodes

    reconfigureNodes.clear()

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(test_node - node)
            if distance <= search_radius:
                reconfigureNodes.append(node)

def find_reconfigurable_nodes_cost(test_node):

    global reconfigureNodesCost

    reconfigureNodesCost.clear()

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(test_node - node)
            if distance <= search_radius:
                cost = calculate_cost(x,y)
                reconfigureNodesCost.append(cost)
    
    #print("costs:")
    #print(reconfigureNodesCost)

def reconfigure_paths(new_path, new_node, lowest_cost):

    global reconfigureNodes
    global reconfigureNodesCost

    reconfigureNodes.clear()
    reconfigureNodesCost.clear()

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(new_node - node)
            if distance <= search_radius:
                original_cost = calculate_cost(x,y)
                compare_cost = lowest_cost + distance
                if compare_cost < original_cost:
                    bind_path(new_path, x, y)
    
    #print("costs:")
    #print(reconfigureNodesCost)
    
def find_lowest_cost(normalized_node, search_radius):
    
    smallest_cost = float('inf')
    smallest_cost_list_index = 0
    smallest_cost_path_index = 0

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(normalized_node - node)
            if distance <= search_radius:
                cost = calculate_node_cost(x, y, normalized_node)
                if cost < smallest_cost:
                    smallest_cost = cost
                    smallest_cost_list_index = x
                    smallest_cost_path_index = y
    
    return cost
    
def find_best_path():

    smallest_distance = float('inf')
    smallest_cost = float('inf')
    list_index = 0
    path_index = 0

    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            distance = np.linalg.norm(endNode - node)
            if distance < smallest_distance:
                smallest_distance = distance
                list_index = x
                
    
    if smallest_distance <= goal_radius:
        for x, path in enumerate(pathList):
            for y, node in enumerate(path):
                distance = np.linalg.norm(endNode - node)
                if distance < goal_radius:
                    cost = calculate_cost(x,y)
                    if cost < smallest_cost:
                        smallest_cost = cost
                        list_index = x
    
    bestPath = pathList[list_index]
    #print(bestPath)

    return bestPath

# -- Draw Functions -- # 

def draw_edges_between_nodes(path, color, screen):
    for i in range(len(path) - 1):
        pygame.draw.line(screen, color, (int(path[i][0]), int(path[i][1])), (int(path[i+1][0]), int(path[i+1][1])), 2)

# -- Path Storage -- #

def add_node_to_path(list_index, new_node):

    global pathList

    # Initialize the list at list_index if it doesn't exist
    while list_index >= len(pathList):
        pathList.append([])

    pathList[list_index].append(new_node)

def add_path_to_list(new_path):

    global pathList

    path = []
    path.append(startNode)
    path.append(new_path)
    
    pathList.append(new_path)

def delete_path(list_index):
    
    global pathList

    del pathList[list_index]

def create_new_path():

    global pathList

    pathList.append(startNode)

def return_list(list_index):
    found_path = pathList[list_index]
    #print(found_path) 

def return_node(list_index, node_index):
    found_path = pathList[list_index]
    found_node = found_path[node_index]
    #print(found_node)

def split_path(nearest_list_index, nearest_path_index, new_node):
    new_path = []
    
    for x, node in enumerate(pathList[nearest_list_index]):
        if x <= nearest_path_index:
            new_path.append(node)

    new_path.append(new_node)
    return new_path

def bind_path(new_path, old_list_index, old_path_index):
    
    global latestPath

    latestPath.clear()

    binded_path = []

    for x, node in enumerate(new_path):
        binded_path.append(node)

    for x, node in enumerate(pathList[old_list_index]):
        if x >= old_path_index:
            binded_path.append(node)
    
    latestPath = binded_path
    
    #print("binded path")

    return binded_path

def calculate_cost_matrix():
    global costList
    costList = []  # Initialize costList as an empty list

    for path in pathList:
        current_cost = 0
        costs = []

        for y, nodes in enumerate(path):
            if y == 0:
                costs.append(current_cost)
            elif y > 0:
                current_cost += np.linalg.norm(path[y] - path[y-1])
                costs.append(current_cost)

        costList.append(costs)  # Append the costs list for each path

def find_nodes_of_interest(new_node, lowest_cost_list_index):
    global nodesOfInterest
    nodesOfInterest = []  # Initialize costList as an empty list

    for x, path in enumerate(pathList):
        indices = []
        for y, node in enumerate(path):
            if np.linalg.norm(node-new_node) < search_radius:
                if y != 0:
                    if x != lowest_cost_list_index:
                        indices.append(x)
                        indices.append(y)
                        nodesOfInterest.append(indices)
                        indices = []
        
    
    print(nodesOfInterest)

def count_nodes():

    nodeCount = 0

    for x, path in enumerate(pathList):
        nodeCount = nodeCount + len(path)
    
    return nodeCount

def generate_paths():
    std_dev = 300  # Random Node Generation Standard Deviation Parameter
    max_growth_radius = 30  # Largest Length a new branch can grow
    search_radius = 75 # radius of nodes included when rewiring branch
    global bestPath

    # Add a node at a random position
    random_node = generate_random_node(std_dev)

    # Find Nearest Node Info
    nearest_list_index = find_nearest_node_list_index(random_node)
    nearest_path_index = find_nearest_node_path_index(random_node)
    nearest_node = find_nearest_node(random_node)

    # create a new node that is within the max growth radius
    new_node = normalize_by_radius(random_node, nearest_node, max_growth_radius)

    for i in listObstacleNodes:
        if np.linalg.norm(new_node - i) <= obstacleRadius:
            return

    # if np.linalg.norm(new_node - obstacleNode) <= obstacleRadius:
    #     return 

    # Find surrounding node cost Info
    lowest_cost_list_index = find_lowest_cost_list_index(new_node, search_radius)
    lowest_cost_path_index = find_lowest_cost_path_index(new_node, search_radius)
    lowest_cost = find_lowest_cost(new_node, search_radius)

    # if the new node is placed at the end of the path:
    if lowest_cost_path_index == len(pathList[nearest_list_index]) - 1:
        add_node_to_path(lowest_cost_list_index, new_node)
        new_path = pathList[lowest_cost_list_index]
        #("node added to existing path")
                
    # if the new node is placed in the middle of the path:
    else:
        new_path = split_path(lowest_cost_list_index, lowest_cost_path_index, new_node)
        add_path_to_list(new_path)
        #print("path split")
                    
        #bestPath = pathList[1]
        bestPath = find_best_path()
    
    # robotPosition = bestPath[robotPositionIndex]

def update_display(screen):
    WHITE = (255, 255, 255)
    BLUE = (200, 200, 255)
    PURPLE = (125, 0, 125)
    GREEN = (0, 255, 0)
    LIGHTGREEN = (200,255,200)
    RED = (255, 0, 0)
    GRAY = (100,100,100)
    # Update the display
    screen.fill(WHITE)

    # Draw Start
    pygame.draw.circle(screen, RED, startNode, 10)

    # Draw Goal
    pygame.draw.circle(screen, LIGHTGREEN, endNode, goal_radius)
    pygame.draw.circle(screen, GREEN, endNode, 30)

    # Draw nodes
    for x, path in enumerate(pathList):
        for y, node in enumerate(path):
            pygame.draw.circle(screen, BLUE, (int(node[0]), int(node[1])), 2)

    for y, node in enumerate(reconfigureNodes):
        pygame.draw.circle(screen, RED, (int(node[0]), int(node[1])), 5)


    for y, node in enumerate(bestPath):
        pygame.draw.circle(screen, PURPLE, (int(node[0]), int(node[1])), 5)
        
    for x, path in enumerate(pathList):
        draw_edges_between_nodes(pathList[x], BLUE, screen)

    draw_edges_between_nodes(bestPath, PURPLE, screen)    

    
    # Draw Obstacles
    for i in listObstacleNodes:
        pygame.draw.circle(screen, GRAY, i, obstacleRadius)

    nodeCount = count_nodes()
    print(nodeCount)
    pygame.display.flip()

def delete_all_nodes():
    global pathList
    pathList = []


def RRT(start, goal, obstacles, width, height, search_radius=75, max_iter=1000):
    pygame.init()

    # Set up display
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Random Node Placement')

    # Colours

    # Start Node
    startNode = np.array(start)

    # End Goal
    endNode = np.array(goal)

    goal_radius = 100
    # Latest Path
    latestPath = []
    # Cost List
    costList = []
    # Best Path
    # nodes of interest 
    nodesOfInterest = []

    betterNodeX = 0
    betterNodeY = 0
    betterNodes = []
    global bestPath
    # Reconfigure Nodes List
    reconfigureNodes = []
    reconfigureNodesCost = []

    # Exploration Parameters
    std_dev = 300  # Random Node Generation Standard Deviation Parameter
    max_growth_radius = 60  # Largest Length a new branch can grow
    search_radius = search_radius # radius of nodes included when rewiring branch

    # Visualisation Parameters
    best_path_index = 0 # color this path green

    # Obstacles

    listObstacleNodes = np.array(obstacles)
    obstacleRadius = 30

    add_node_to_path(0, startNode)
    bestPath = pathList[0]

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    delete_all_nodes()
                    add_path_to_list([startNode])
        
        nodeCount = count_nodes()
        
        while(nodeCount < max_iter):
            nodeCount = count_nodes()
            generate_paths()
            update_display(screen)
                        
        nodeCount = count_nodes()
        print(nodeCount)
        

        pygame.display.flip()
        return bestPath



#############################TESTING PARAMS##############################################################
WIDTH, HEIGHT = 256, 256
startNode = np.array([0, 0])
endNode = np.array([250,256])
obstacleRadius = 10
search_radius = 50
listObstacleNodes = np.array([[20,20], [40,40], [50,50], [60, 60]])
path = RRT(startNode, endNode, listObstacleNodes, WIDTH, HEIGHT)
print(path)


"""
640/480 -> 256/x
x = 192


"""


#YOLO = [[1,2, 'PERSON'], [1,3, 'TENNIS BALL']]
#RRT(YOLO) = [[X,Y], [X,Y]...]
#PYTHON TO TRANSMITTER ARDUINO = [int(x), int(y)] 8 bit binary for each x,y, 7 bits, 1 pos neg
"""
for i in rrt:
    nodex1, nodey1 = 000001, 0000100
    serialwrite(toarduino)

once all these binarys of nodes gets to transmitter, then binary flag to send to reciever
"""


#TRANSMITTER TO RECIEVER = BINARY -> BINARY  