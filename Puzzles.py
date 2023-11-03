
import argparse
import heapq
from typing import List, Tuple, Union


# Define the heuristic function (Sum of Manhattan distance for each node)
def manhattan_distance(state, goal):
    sum_distance = 0
    for x in range(3):
        for y in range(3):
            for z in range(3):
                value = state[x][y][z]
                if value != 0:
                    goal_x, goal_y, goal_z = find_tile_position(goal, value)
                    sum_distance += abs(x - goal_x) + abs(y - goal_y) + abs(z - goal_z)
    return sum_distance

# Helper function to find the position of a tile in the grid
def find_tile_position(state, tile):
    for x in range(3):
        for y in range(3):
            for z in range(3):
                if state[x][y][z] == tile:
                    return x, y, z

# Define the A* search node class
class Node:
    def __init__(self, state: List[List[List[int]]], depth: int, cost: int, parent=None):
        self.state = state
        self.depth = depth
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

# Define the A* search algorithm
def astar_search(initial_state, goal_state):
    # use heap to automatically store the order from least cost to largest cost
    # open_set = []

    # initial_node = Node(initial_state, 0, 0)
    # heapq.heappush(open_set, initial_node)
    pass

# Main function
def main() -> None:
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    cmdline = parser.parse_args()

    # Read input file and create initial and goal states
    with open(cmdline.filename, 'r') as file:
        lines = file.read().splitlines()

    initial_state = [list(map(int, line.split())) for line in lines[:11]]  # Initial state
    goal_state = [list(map(int, line.split())) for line in lines[12:24]]  # Goal state

    # Find the solution using A* search
    

if __name__ == "__main__":
    main()
