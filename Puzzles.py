
import argparse
import heapq
from typing import List, Tuple, Union
import copy


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

def find_reachable_states(state):
    # Possible actions: East (E), West (W), North (N), South (S), Up (U) and Down (D)
    reachable_states = []
    reachable_states_actions = []
    # East:
    if find_tile_position(state, 0)[2] != 2: # not in the third column of any level of tiles
        reachable_states += [get_state(state, "East")]
        reachable_states_actions += ["East"]
    # West:
    if find_tile_position(state, 0)[2] != 0: # not in the first column of any level of tiles
        reachable_states += [get_state(state, "West")]
        reachable_states_actions += ["West"]
    # North:
    if find_tile_position(state, 0)[1] != 0: # not in the first row of any level of tiles
        reachable_states += [get_state(state, "North")]
        reachable_states_actions += ["North"]
    # South:
    if find_tile_position(state, 0)[1] != 2: # not in the third row of any level of tiles
        reachable_states += [get_state(state, "South")]
        reachable_states_actions += ["South"]
    # Up:
    if find_tile_position(state, 0)[0] != 0: # not in the first level of tiles
        reachable_states += [get_state(state, "Up")]
        reachable_states_actions += ["Up"]
    # Down:
    if find_tile_position(state, 0)[0] != 2: # not in the third level of tiles
        reachable_states += [get_state(state, "Down")]
        reachable_states_actions += ["Down"]

    return reachable_states, reachable_states_actions

def get_state(state, action):
    print("this is what i got in")
    print(state)
    tile_position = find_tile_position(state, 0)
    new_state = copy.deepcopy(state)
    if action == "East": # go to the right, possible sample input: ([0, 1, 2] or [1, 0, 2])
        if tile_position[2] == 0: # [0, 1, 2]
            # new_state = state
            new_state[tile_position[0]][tile_position[1]] = [state[tile_position[0]][tile_position[1]][1], 0, state[tile_position[0]][tile_position[1]][2]]
        elif tile_position[2] == 1: # [1, 0, 2]
            # new_state = state
            new_state[tile_position[0]][tile_position[1]] = [state[tile_position[0]][tile_position[1]][0], state[tile_position[0]][tile_position[1]][2], 0]
    elif action == "West": # go to the left, possible sample input: ([1, 0, 2] or [1, 2, 0])
        if tile_position[2] == 1: # [1, 0, 2]
            # new_state = state
            new_state[tile_position[0]][tile_position[1]] = [0, state[tile_position[0]][tile_position[1]][0], state[tile_position[0]][tile_position[1]][2]]
        elif tile_position[2] == 2: # [1, 2, 0]
            # new_state = state
            new_state[tile_position[0]][tile_position[1]] = [state[tile_position[0]][tile_position[1]][0], 0, state[tile_position[0]][tile_position[1]][1]]
    elif action == "South": # slide up on the same level
        # new_state = state
        print("south!!")
        new_state[tile_position[0]][tile_position[1]][tile_position[2]] = new_state[tile_position[0]][tile_position[1]+1][tile_position[2]]
        new_state[tile_position[0]][tile_position[1]+1][tile_position[2]] = 0
    elif action == "North": # slide down on the same level
        # new_state = state
        new_state[tile_position[0]][tile_position[1]][tile_position[2]] = new_state[tile_position[0]][tile_position[1]-1][tile_position[2]]
        new_state[tile_position[0]][tile_position[1]-1][tile_position[2]] = 0
    elif action == "Up": # go up a level, possible input: on level 1 or 2
        # new_state = state
        new_state[tile_position[0]][tile_position[1]][tile_position[2]] = state[tile_position[0]-1][tile_position[1]][tile_position[2]]
        new_state[tile_position[0]-1][tile_position[1]][tile_position[2]] = 0
    elif action == "Down": # go down a level, possible input: on level 0 or 1
        # new_state = state
        new_state[tile_position[0]][tile_position[1]][tile_position[2]] = state[tile_position[0]+1][tile_position[1]][tile_position[2]]
        new_state[tile_position[0]+1][tile_position[1]][tile_position[2]] = 0
    else:
        raise Exception("Invalid action")
    return new_state

# Define the A* search node class
class Node:
    def __init__(self, state: List[List[List[int]]], depth: int, cost: int, parent, action):
        self.state = state
        self.depth = depth
        self.cost = cost
        self.parent = parent
        self.action = action

    def __lt__(self, other):
        return self.cost < other.cost

# Define the A* search algorithm
def astar_search(initial_state, goal_state):
    #DECLARE A NODE: Node(state, depth, cost, parent)
    initial_node = Node(initial_state, 0, 0, None, None)

    frontier = [initial_node] # current frontier
    reached = [initial_node.state] # reached states

    # while len(frontier) > 0:
    for i in range(5):
        #get node in frontier with least cost
        current_cost = 10000000000000
        for i in range(len(frontier)): # loop through the nodes in the frontier
            if frontier[i].cost + frontier[i].depth < current_cost:
                current_node = frontier[i]
                current_node_index = i
        frontier.pop(current_node_index) # pop the current_node from the frontier
        print("popped state")
        print(current_node.state)
        if current_node.state == goal_state: # check goal state before expansion
            print("solution found!!!")
            return current_node
        reachable_states, reachable_states_actions = find_reachable_states(current_node.state) # expand
        print(reachable_states_actions)
        for i in reachable_states:
            print(i)
        for i in range(len(reachable_states_actions)):
            if reachable_states[i] not in reached:
                new_node = Node(reachable_states[i], 1, manhattan_distance(reachable_states[i], goal_state), parent = current_node, action = reachable_states_actions[i])
                frontier.append(new_node)
                reached.append(new_node.state)
                print("appended")
    print("solution not found!!")
    return None

# def expand(node, actions):
#     state = node.state
#     for i in actions:
#         new_node()
        
# Main function
def main() -> None:
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    cmdline = parser.parse_args()

    # Read input file and create initial and goal states
    with open(cmdline.filename, 'r') as file:
        lines = file.read().splitlines()

    # initial_state = [list(map(int, line.split())) for line in lines[:11]]  # Initial state
    # goal_state = [list(map(int, line.split())) for line in lines[12:24]]  # Goal state

    initial_state = [[list(map(int, line.split())) for line in lines[:3]]]  # Initial state
    initial_state += [[list(map(int, line.split())) for line in lines[4:7]]]
    initial_state += [[list(map(int, line.split())) for line in lines[8:11]]]

    goal_state = [[list(map(int, line.split())) for line in lines[12:15]]]  # Goal state
    goal_state += [[list(map(int, line.split())) for line in lines[16:19]]]
    goal_state += [[list(map(int, line.split())) for line in lines[20:23]]]

    # Find the solution using A* search
    print(astar_search(initial_state, goal_state))
    print("intital state:")
    print(initial_state)
    # print("goal state")
    # print(goal_state)
    # print("find tile position")
    # print(find_tile_position(initial_state, 22))
    # print("find manhattan")
    # print(manhattan_distance(initial_state, goal_state))
    # print("reachable states")
    # r, a = find_reachable_states(initial_state)
    # for i in range(len(r)):
    #     print(r[i])
    #     print(a[i])
    #     print("-----------")
    # print("go south")
    # print(get_state(initial_state, "South"))

if __name__ == "__main__":
    main()
