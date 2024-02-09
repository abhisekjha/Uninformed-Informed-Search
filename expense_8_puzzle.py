import sys
import heapq
from queue import Queue
from datetime import datetime


#print and write the result to the file
def write_results_to_file(filename, nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path):
    try:
        with open(filename, "a") as file:
            file.write("Nodes Popped: {}\n".format(nodes_popped))
            file.write("Nodes Expanded: {}\n".format(nodes_expanded))
            file.write("Nodes Generated: {}\n".format(nodes_generated))
            file.write("Max Fringe Size: {}\n".format(max_fringe_size))
            file.write("Solution Found at depth {} with cost of {}\n".format(depth, cost))
            file.write("Steps:\n{}\n".format('\n'.join(path)))
    except Exception as e:
        print("An error occurred:", str(e))

#print results
def print_results(nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path):
    print("Nodes Popped:", nodes_popped)
    print("Nodes Expanded:", nodes_expanded)
    print("Nodes Generated:", nodes_generated)
    print("Max Fringe Size:", max_fringe_size)
    print("Solution Found at depth", depth, "with cost of", cost)
    print("Steps:\n", '\n'.join(path))

def goal(state, goal_state):
    return state == goal_state

def find_empty_cell(current_state):
    for i, row in enumerate(current_state):
        for j, val in enumerate(row):
            if val == 0:
                return i, j

def moveto(current_state, row, col, drow, dcol):
    new_state = [list(row) for row in current_state]
    new_state[row][col], new_state[row + drow][col + dcol] = new_state[row + drow][col + dcol], new_state[row][col]
    return new_state

# Breadth-First Search Algorithm
def bfs_algorithm(initial_state, goal_state, save_trace=False):
    q = Queue()
    visited = set()
    nodes_popped = 0; nodes_expanded = 0; nodes_generated = 1; max_fringe_size = 1; depth = 0

    moves = {
        'up': (-1, 0),
        'down': (1, 0), 
        'left': (0, -1),
        'right': (0, 1)
        }
    

    print("***Algorithm used is BFS***\n")

    if save_trace:
        filename = "tracedatetime.txt"
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        #open the DateTime file and same the trace
        with open(filename, "w") as file:
            file.write(f"Search trace ({now})\n\n")

    q.put((initial_state, [], 0))

    while not q.empty():
        current, path, cost = q.get()
        nodes_popped += 1
        visited.add(str(current))

        if current == goal_state:
            if save_trace:
                filename = "tracedatetime.txt"
                write_results_to_file(filename, nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            else:
                print_results(nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            return True

        row, col = find_empty_cell(current)

        for action, (drow, dcol) in moves.items():
            new_row, new_col = row + drow, col + dcol
            if 0 <= new_row < 3 and 0 <= new_col < 3:
                new_state = moveto(current, row, col, drow, dcol)
                if str(new_state) not in visited:

                    nodes_generated += 1
                    cost_to_move = new_state[row][col]
                    path_copy = path.copy()
                    path_copy.append("Move {} {} ".format(current[new_row][new_col], action.upper()))
                    q.put((new_state, path_copy, cost + cost_to_move))

        nodes_expanded += 1
        if q.qsize() > max_fringe_size:
            max_fringe_size = q.qsize()
        if cost > depth:
            depth = cost

        if save_trace:
            with open(filename, "a") as file:

                file.write("Iteration {}\n".format(nodes_expanded))
                file.write("Closed: {}\n".format(str(visited)))
                file.write("Fringe: [\n")
                for item in list(q.queue):
                    file.write("\t{}\n".format(str(item)))
                file.write("]\n\n")

    return False

# Uniform Cost Search Algorithm
def ucs_algorithm(initial_state, goal_state, save_trace=False):
    
    queue = [(0, initial_state, [], 0)]
    visited = set()

    nodes_popped = 0; nodes_expanded = 0; nodes_generated = 1; max_fringe_size = 1; depth = 0
    moves = {'up': (-1, 0),
            'down': (1, 0), 
            'left': (0, -1), 
            'right': (0, 1)
            }
    
    

    print("***Algorithm used is UCS***")

    while queue:
        _, current, path, cost = heapq.heappop(queue)
        nodes_popped += 1
        visited.add(str(current))

        if current == goal_state:
            if save_trace:
                filename= "tracedatetime.txt "
                write_results_to_file(filename, nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            else:
                print_results(nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            return True

        row, col = find_empty_cell(current)
        for action, (drow, dcol) in moves.items():

            new_row, new_col = row + drow, col + dcol

            if 0 <= new_row < 3 and 0 <= new_col < 3:
                new_state = moveto(current, row, col, drow, dcol)

                if str(new_state) not in visited:
                    nodes_generated += 1
                    cost_to_move = new_state[row][col]
                    path_copy = path.copy()
                    path_copy.append("Move {} {} ".format(current[new_row][new_col], action.upper()))
                    heapq.heappush(queue, (cost + cost_to_move, new_state, path_copy, cost + cost_to_move))

        nodes_expanded += 1
        if len(queue) > max_fringe_size:
            max_fringe_size = len(queue)

        if cost > depth:
            depth = cost

    if save_trace:
        with open("tracedatetime.txt ", "a") as file:
            file.write("Closed: {}\n".format(str(visited)))
            file.write("Fringe: [\n")

            for item in queue:
                file.write("\t{}\n".format(str(item)))
            file.write("]\n\n")

    return False

# Heuristic function for A* algorithm
def calculate_heuristic(start_state, goal_state):

    start_state = [num for row in start_state for num in row]
    goal_state = [num for row in goal_state for num in row]
    heuristic_distance = 0

    for i in range(9):
        if start_state[i] != goal_state[i]:
            (goal_i, goal_j) = divmod(goal_state.index(start_state[i]), 3)
            (i, j) = divmod(i, 3)
            
            heuristic_distance += abs(i - goal_i) + abs(j - goal_j)

    return heuristic_distance

# Greedy Algorithm
def greedy_search(start_state, goal_state, save_trace=False):
    q = []
    visited = set()
    moves = {'up': (-1, 0), 
             'down': (1, 0), 
             'left': (0, -1), 
             'right': (0, 1)
             }
    
    nodes_popped = 0; nodes_expanded = 0; nodes_generated = 1; max_fringe_size = 1; depth = 0; cost = 0

    priority = calculate_heuristic(start_state, goal_state)
    heapq.heappush(q, (priority, start_state, [], cost))

    print("**Algorithm used is Greedy**")

    if save_trace:
        filename = "tracedatetime.txt"
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        with open(filename, "w") as file:
            file.write(f"Search trace ({now})\n\n")

    while q:
        _, current, path, cost = heapq.heappop(q)
        nodes_popped += 1
        visited.add(str(current))

        if current == goal_state:
            if save_trace:
                filename= "tracedatetime.txt"
                write_results_to_file(filename, nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            else:
                print_results(nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            return True

        row, col = find_empty_cell(current)
        for action, (drow, dcol) in moves.items():

            new_row, new_col = row + drow, col + dcol
            if 0 <= new_row < 3 and 0 <= new_col < 3:

                new_state = moveto(current, row, col, drow, dcol)
                if str(new_state) not in visited:

                    nodes_generated += 1
                    cost_to_move = new_state[row][col]
                    path_copy = path.copy()
                    path_copy.append("Move {} {} ".format(current[new_row][new_col], action.upper()))
                    new_cost = cost + cost_to_move

                    new_priority = calculate_heuristic(new_state, goal_state)
                    heapq.heappush(q, (new_priority, new_state, path_copy, new_cost))

        nodes_expanded += 1

        if len(q) > max_fringe_size:
            max_fringe_size = len(q)

        if cost > depth:
            depth = cost

        if save_trace:
            with open(filename, "a") as file:
                file.write("Closed: {}\n".format(str(visited)))
                file.write("Fringe: [\n")

                for item in q:
                    file.write("\t{}\n".format(str(item)))
                file.write("]\n\n")

    return False

# A* algorithm
def a_star_algorithm(initial_state, goal_state, save_trace=False):
    q = []
    visited = set()
    nodes_popped = 0; nodes_expanded = 0; nodes_generated = 1; max_fringe_size = 1

    moves = {'up': (-1, 0), 
             'down': (1, 0), 
             'left': (0, -1), 
             'right': (0, 1)}
    
    depth = 0; cost = 0

    priority = cost + calculate_heuristic(initial_state, goal_state)
    heapq.heappush(q, (priority, initial_state, [], cost))

    print("**Algorithm used is (A*) ***")

    while q:
        _, current, path, cost = heapq.heappop(q)
        nodes_popped += 1
        visited.add(str(current))

        if current == goal_state:
            if save_trace:
                filename= "tracedatetime.txt"
                write_results_to_file(filename, nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
                
            else:
                print_results(nodes_popped, nodes_expanded, nodes_generated, max_fringe_size, depth, cost, path)
            return True

        row, col = find_empty_cell(current)
        for action, (drow, dcol) in moves.items():
            new_row, new_col = row + drow, col + dcol

            if 0 <= new_row < 3 and 0 <= new_col < 3:
                new_state = moveto(current, row, col, drow, dcol)

                if str(new_state) not in visited:
                    nodes_generated += 1
                    cost_to_move = new_state[row][col]
                    path_copy = path.copy()
                    path_copy.append("Move {} {} ".format(current[new_row][new_col], action.upper()))
                    new_cost = cost + cost_to_move
                    new_priority = new_cost + calculate_heuristic(new_state, goal_state)
                    heapq.heappush(q, (new_priority, new_state, path_copy, new_cost))

        nodes_expanded += 1

        if len(q) > max_fringe_size:
            max_fringe_size = len(q)
            
        if cost > depth:
            depth = cost

    if save_trace:
        with open("tracedatetime.txt", "a") as file:
            file.write("Closed: {}\n".format(str(visited)))
            file.write("Fringe: [\n")
            for item in q:
                file.write("\t{}\n".format(str(item)))
            file.write("]\n\n")

    return False

def main():
    if len(sys.argv) < 3:
        print("Invalid Command Line argument. Please use the following format: python3 expense_8_puzzle.py <start-file> <goal-file> <method> <dump-flag>\nUseful Info.: Format for methods:\n")
        print("Breadth-First Search=bfs,\nUniform Cost Search = ucs,\nGreedy Algorith =greedy,\nA*= astar\n")
    else:
        arg1 = sys.argv[1] if len(sys.argv) > 1 else 'start.txt'
        arg2 = sys.argv[2] if len(sys.argv) > 2 else 'goal.txt'
        arg3 = sys.argv[3] if len(sys.argv) > 3 else 'astar'
        arg4 = sys.argv[4] if len(sys.argv) > 4 else 'False'

        with open(arg1, 'r') as file:
            file_contents = file.read()

        rows = file_contents.split('\n')
        start_state = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        for i in range(3):
            numbers = rows[i].split(' ')
            for j in range(3):
                start_state[i][j] = int(numbers[j])

        with open(arg2, 'r') as file:
            file_contents = file.read()

        rows = file_contents.split('\n')
        goal_state = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]

        for i in range(3):
            numbers = rows[i].split(' ')
            for j in range(3):
                goal_state[i][j] = int(numbers[j])

        save_trace = True if arg4.lower() == 'true' else False

        if arg3 == 'bfs':
            if save_trace:
                bfs_algorithm(start_state, goal_state, save_trace=True)
            else:
                bfs_algorithm(start_state, goal_state)
        elif arg3 == 'greedy':
            if save_trace:
                greedy_search(start_state, goal_state, save_trace=True)
            else:
                greedy_search(start_state, goal_state)
        elif arg3 == 'ucs':
            if save_trace:
                ucs_algorithm(start_state, goal_state, save_trace=True)
            else:
                ucs_algorithm(start_state, goal_state)
        elif arg3 == 'astar':
            if save_trace:
                a_star_algorithm(start_state, goal_state, save_trace=True)
            else:
                a_star_algorithm(start_state, goal_state)
        else:
            if save_trace:
                a_star_algorithm(start_state, goal_state, save_trace=True)
            else:
                a_star_algorithm(start_state, goal_state)

if __name__ == '__main__':
    main()
