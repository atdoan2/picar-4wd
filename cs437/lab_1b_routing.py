import numpy as np
import time
import picar_4wd as fc
import lab_1b_more_advanced_mapping as slam
from AStartSearch import AStar

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

def run():
    a_star_search = AStar()
    threshold = 100  # Set threshold (can adjust as needed)
    while True:
        updated_map = slam.update_map(picar_position, threshold)
        buffered_map = a_star_search.add_buffer(a_star_search.add_buffer(a_star_search.add_buffer(updated_map)))
        
        for row in buffered_map:
            for elem in row:
                print(elem,end="")
            print()

        start = (map_height-1,map_width/2)
        goal = (0,map_width/2)
        
        if current_angle == 180 or current_angle == -180:
            path, move_directions = a_star_search.astar_search(buffered_map, start, goal)
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
