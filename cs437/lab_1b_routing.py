import numpy as np
import time
import picar_4wd as fc
import lab_1b_more_advanced_mapping as slam

import heapq

# Initialize the map
map_width = 100
map_height = 100
picar_map = np.zeros((map_width, map_height), dtype=int)

# Initialize picar's positioning as well as its speed for movement/turning
picar_position = {
    'x': 0,
    'y':50,
    'angle': 0
}

velocity = {
    'linear': 0.1,
    'turning': 5
}

servo_step_angle = 5
current_angle = -180
us_step = servo_step_angle

movements = [(1, 0, "down"), (-1, 0, "up"), (0, 1, "right"), (0, -1, "left")]

def heuristic(current, goal):
    # Calculate the Manhattan distance as the heuristic
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

def astar_search(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    open_set = []  # Priority queue of nodes to be evaluated
    closed_set = set()  # Set of nodes already evaluated
    came_from = {}  # Dictionary to store the path
    move_directions = {}  # Dictionary to store moves
    
    # Initialize the open set with the starting node
    heapq.heappush(open_set, (0, start))
    
    # Initialize scores for each node to infinity
    g_score = {position: float('inf') for row in grid for position in row}
    g_score[start] = 0
    
    # Main A* loop
    while open_set:
        current_g, current_node = heapq.heappop(open_set)
        
        if current_node == goal:
            # Reconstruct the path if the goal is reached
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path = path[::-1]  # Return the path in the correct order
            return path, move_directions
        
        closed_set.add(current_node)
        
        for dr, dc, direction in movements:
            r, c = current_node[0] + dr, current_node[1] + dc
            neighbor = (r, c)
            
            c = int(c)
            
            if 0 <= r < rows and 0 <= c < cols and grid[r][c] != 1 and neighbor not in closed_set:
                tentative_g_score = g_score[current_node] + 1
                
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current_node
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    
                    # Store the move direction
                    move_directions[neighbor] = direction
    
    return None, None  # If no path is found

def add_buffer(grid):
    rows, cols = len(grid), len(grid[0])
    new_grid = [[0 for _ in range(cols)] for _ in range(rows)]

    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                # Set the current cell to 1
                new_grid[r][c] = 1

                # Set neighboring cells to 1 (within bounds)
                for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        new_grid[nr][nc] = 1

    return new_grid

def run():
    threshold = 100  # Set threshold (can adjust as needed)
    while True:
        updated_map = slam.update_map(picar_position, threshold)
        buffered_map = add_buffer(add_buffer(add_buffer(updated_map)))
        
        for row in buffered_map:
            for elem in row:
                print(elem,end="")
            print()

        start = (map_height-1,map_width/2)
        goal = (0,map_width/2)
        
        
        if current_angle == 180:
            path, move_directions = astar_search(buffered_map, start, goal)
            if path:
                moves = list(move_directions.values())
                moves = moves[0:3]
                print(moves)
                for move in moves:
                    if move == "up":
                        print("move forward")
                        fc.forward(3)
                        time.sleep(1)
                        fc.stop()
                    elif move == "down":
                        print("move backward")
                        fc.backward(3)
                        time.sleep(1)
                        fc.stop()
                    elif move == "left":
                        print("turn left")
                        fc.turn_left(20)
                        time.sleep(1)
                        print("move forward")
                        fc.forward(20)
                        time.sleep(1)
                        fc.stop()
                    elif move == "right":
                        print("turn right")
                        fc.turn_right(20)
                        time.sleep(1)
                        print("move forward")
                        fc.forward(20)
                        time.sleep(1)
                        fc.stop()
                time.sleep(5)

        
       # update_car_position(picar_position, velocity)
        #(picar_position)

if __name__ == "__main__":
    try:
        run()
    finally:
        fc.stop()
