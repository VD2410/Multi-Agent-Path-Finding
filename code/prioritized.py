import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []


        # Constraint for Task 1.2
        # constraints = [{'agent': 0,'loc': [(1,5)],'timestep': 4}]
        
        # Constraint for Task 1.3
        # constraints = [{'agent': 1,'loc': [(1,2),(1,3)],'timestep': 1}]
        
        # Constraints for Task 1.4
        # constraints = [{'agent': 0,'loc': [(1,5)],'timestep': 10},
        #                 {'agent': 0,'loc': [(1,3),(1,4)],'timestep': 5},
        #                 {'agent': 0,'loc': [(1,3),(1,4)],'timestep': 7},
        #                 {'agent': 0,'loc': [(1,2),(1,3)],'timestep': 2}]


        # Constraints for Task 1.5
        # constraints = [{'agent': 1,'loc': [(1,4)],'timestep': 2},
        #                 {'agent': 1,'loc': [(1,3),(1,2)],'timestep': 2}]


        for i in range(self.num_of_agents):  # Find path for each agent

            
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)


            if path is None:
                raise BaseException('No solutions')
            result.append(path)



            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################


            # if len(constraint_table.keys()) != 0:
            #     if next_time > max(constraint_table.keys()):
            #         next_time = max(constraint_table.keys())

            # print(path)

            for next_agent in range(self.num_of_agents):


                for i in range (0,10):

                #     # Task 2.3 Adding goal constraints and max timestep allowed was 10

                    constraints.append({'agent' : next_agent, 'loc' : [path[len(path) - 1]], 'timestep' : len(path)+i - 1})
                
                for nPath in range(len(path)):
                    # print(path[len(path) - 1])

                    # constraints.append({'agent' : next_agent, 'loc' : [path[len(path) - 1]], 'timestep' : nPath})

                    # constraints.append({'agent' : next_agent, 'loc' : [path[len(path) - 1]], 'timestep' : nPath+len(path)})

                    # Task 2.1 Vertex Constraints
                    if next_agent != i:

                        constraints.append({'agent' : next_agent, 'loc' : [path[nPath]], 'timestep' : nPath})


                        # Task 2.2 Edge Constraints

                        if nPath > 0:
                            constraints.append({'agent' : next_agent, 'loc' : [path[nPath],path[nPath-1]], 'timestep' : nPath})



        self.CPU_time = timer.time() - start_time
        # print(constraints)

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
