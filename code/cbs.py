import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost

import copy



def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst



def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    timestep_total = max(len(path1),len(path2))

    collision = dict()

    for t in range (0,timestep_total):
        # Edge collision
        if t!=0 and get_location(path1,t-1) == get_location(path2,t) and get_location(path1,t) == get_location(path2,t-1):
            collision = {'loc' : [get_location(path2,t-1),get_location(path2,t)],'timestep' : t}
        # Vertex collision
        if get_location(path1,t) == get_location(path2,t):
            collision = {'loc' : [get_location(path1,t)], 'timestep' : t}

    return collision


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    #  Task 3.1

    agents = len(paths)

    collisions = list()

    for a1 in range(0,agents-1):
        for a2 in range (a1+1,agents):
            collision_loc = detect_collision(paths[a1],paths[a2])
            if collision_loc:
                collisions.append({'a1' : a1, 'a2' : a2, 'loc' : collision_loc['loc'], 'timestep' : collision_loc['timestep']})

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    # Task 3.2
    loc = collision['loc']
    timestep = collision['timestep']

    if len(loc)>1:
        first_constraint = {'agent' : collision['a1'], 'loc' : [loc[1],loc[0]], 'timestep' : timestep}
        second_constraint = {'agent' : collision['a2'], 'loc' : [loc[0],loc[1]], 'timestep' : timestep}
        return [first_constraint,second_constraint]

    if len(loc) == 1:
        first_constraint = {'agent' : collision['a1'], 'loc' : [loc[0]], 'timestep' : timestep}
        second_constraint = {'agent' : collision['a2'], 'loc' : [loc[0]], 'timestep' : timestep}
        return [first_constraint,second_constraint]




def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly



    loc = collision['loc']
    timestep = collision['timestep']


    if random.randint(0,1):
        random_agent = 'a1'
    else:
        random_agent = 'a2'

    if len(loc)>1 and random_agent == 'a1':

        first_constraint = {'agent' : collision['a1'], 'loc' : [loc[1],loc[0]], 'timestep' : timestep, 'positive' : True}
        second_constraint = {'agent' : collision['a1'], 'loc' : [loc[1],loc[0]], 'timestep' : timestep, 'positive' : False}
        return [first_constraint,second_constraint]

    if len(loc)>1 and random_agent == 'a2':
        first_constraint = {'agent' : collision['a2'], 'loc' : [loc[0],loc[1]], 'timestep' : timestep, 'positive' : True}
        second_constraint = {'agent' : collision['a2'], 'loc' : [loc[0],loc[1]], 'timestep' : timestep, 'positive' : False}
        return [first_constraint,second_constraint]

    if len(loc) == 1:
        first_constraint = {'agent' : collision[random_agent], 'loc' : [loc[0]], 'timestep' : timestep, 'positive' : True}
        second_constraint = {'agent' : collision[random_agent], 'loc' : [loc[0]], 'timestep' : timestep, 'positive' : False}
        return [first_constraint,second_constraint]


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # Task 3.3 and Task 3.4
        standard = False
        disjoint = True
        

        while len(self.open_list) > 0:

            new_node = self.pop_node()

            if len(new_node['collisions']) == 0:
            
                print("No collision")
            
                return new_node['paths']

            collision = new_node['collisions'][0]



            if standard == True:
                constraints = standard_splitting(collision)
            if disjoint == True:
                constraints = disjoint_splitting(collision)



            for constraint in constraints:
            
                Q={'constraints' : [],
                    'paths' : [],
                    'cost' : [],
                    'collisions' : []}

                for i in new_node['constraints']:

                    Q['constraints'].append(i)
            
                Q['constraints'].append(constraint)

                for i in new_node['paths']:

                    Q['paths'].append(i)
            
                agent = constraint['agent']
            
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],agent, Q['constraints'])

                if path is not None:
            
                    # raise BaseException('No solutions')

                    Q['paths'][agent] = path

                    paths_violate = list()

                    # Use of paths_violate_constraint

                    if constraint['positive'] == True:

                        paths_violate = paths_violate_constraint(constraint,Q['paths'])

                    for new_agent in paths_violate:

                        new_constraint = {'agent' : new_agent, 'loc' : constraint['loc'], 'timestep' : constraint['timestep'], 'positive' : False}

                        Q['constraints'].append(new_constraint)

                        new_path = a_star(self.my_map, self.starts[new_agent], self.goals[new_agent], self.heuristics[new_agent],new_agent, Q['constraints'])

                        if new_path is None:

                            break

                        Q['paths'][new_agent] = new_path

                    Q['cost'] = get_sum_of_cost(Q['paths'])

                    Q['collisions'] =  detect_collisions(Q['paths'])


                    self.push_node(Q)

            
        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
