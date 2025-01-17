import numpy as np

class idastar(AlgorithmBase):

    WEIGHT = 0.5
    HEURISTIC = 'euclidean' # other option is 'manhattan'

    def execute(self):
        """
        simulates Iterative Deepening A* (A-Star) with the f-value function as:
        f(node) = g(node) + WEIGHT * h(node)

        Function to simulate Depth Bounded Depth First Search is called recursively with increasing depth bound
        
        Assumptions:
            Works for a single start node and single goal node
            Edges have non-negative weights

        Hyperparamters:
            WEIGHT: ratio of weight of h-value to g-value of a node. Used to calculate f-value
            HEURISTIC: metric to calculate h(node) w.r.t goal node. Allowed values ['euclidean', 'manhattan']
        """
        # init
        start = self.start_nodes[0]
        goal = self.goal_nodes[0]
        depthBound = 0
        cheapestUnexpanded = self.heuristic(start, goal, f_name = self.HEURISTIC) * self.WEIGHT      # f-value of start node

        while (self.found_goal == False):
            depthBound = cheapestUnexpanded
            self.show_info('For depth bound: %s' %depthBound)

            # set bound to f(node) of cheapest unexpanded node on OPEN
            cheapestUnexpanded = self.dbdfs(start, goal, depthBound)

        if self.found_goal:
            self.show_info('Path found at (cost) Depth %s' % depthBound)
        else:
            self.show_info('No path available')

    
    def dbdfs(self, start, goal, depthBound):
        """
        simulates Depth Bounded Depth First Search

        if a path is found:
            Path will be printed
            self.found_goal = True will be set
        
        Assumptions:
            Edges have non-negative weights

        Args:
            start: A Node object for the starting Node
            goal: A Node object for the goal Node
            depthBound: An integer/float value of the depth limit we are exploring

        Returns:
            count: An integer value of number of nodes in the constructed path
        """
        # init
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        
        g={i:np.inf  for i in self.get_nodes()}
        g[start]=0
        f={start:self.heuristic(start, goal, f_name = self.HEURISTIC)*self.WEIGHT}
        
        nextDepth = depthBound
        
        self.show_info('Starting DB-DFS')
        
        # kernel
        while True:
            self.alg_iteration_start()

            # find first node in OPEN with f-value under the bound
            nodeIndex = next((index for index, node in enumerate(queue) if f[node] <= depthBound), -2)
            
            # if not found, update depthBound and break
            if nodeIndex == -2:
                nextNode = min(queue, key=lambda x: f[x])
                nextDepth = f[nextNode]
                # self.show_info('Next Depth will be: {}'.format(nextDepth))
                self.alg_iteration_end()
                break
            
            node = queue.pop(nodeIndex)
            visited.append(node)

            # check for GOAL node
            if node == goal:
                visited.append(node)
                self.found_goal=True
                break

            # iterate over children
            index = 0
            for neighbor in self.neighbors(node):
                if neighbor not in visited:
                    # remove from OPEN, if exists
                    if neighbor in queue:
                        queue.remove(neighbor)

                    # update g-val (if updated, set parent)
                    t_g = g[node] + self.get_edge_weight(node,neighbor)
                    if t_g < g[neighbor]:
                        self.set_parent(neighbor,node)
                        g[neighbor] = t_g
                    
                    # update f-val
                    f[neighbor] = g[neighbor] + self.heuristic(neighbor, goal, f_name = self.HEURISTIC)*self.WEIGHT

                    # add to OPEN
                    queue.insert(index, neighbor)
                    index = index + 1

            self.alg_iteration_end()

        if self.found_goal:
            path=self.gen_path()
            self.show_path(path)
        # else:
        #     self.show_info('No path available upto Depth {}'.format(depthBound))
        
        # uncomment to print f, g, h values of nodes in OPEN
        # for i in queue:
        #     self.show_info('Node: g: {}, h: {}, f: {}'.format(g[i], self.heuristic(neighbor, goal, f_name = self.HEURISTIC)*self.WEIGHT, f[i]))

        queue.clear()
        visited.clear()
        return nextDepth