class dfid(AlgorithmBase):

    def execute(self):
        """
        simulates Depth First Iterative Deepening

        Function to simulate Depth Bounded Depth First Search is called recursively with increasing depth bound
        
        Assumptions:
            Works for a single start node and single goal node
            Edges weights are not considered
        """
        # init
        count = -1
        previousCount = 0
        depthBound = 0
        start = self.start_nodes[0]
        goal = self.goal_nodes[0]

        # repeat with incrementing Depth until either Goal is found OR no new node is added
        while (previousCount != count) and (self.found_goal == False):
            previousCount = count
            depthBound = depthBound + 1
            self.show_info('For depth bound: %s' %depthBound)
            count = self.dbdfs(start, goal, depthBound)

        if self.found_goal:
            self.show_info('Path found at Depth %s' % depthBound)
        else:
            self.show_info('No path available')

    
    def dbdfs(self, start, goal, depthBound):
        """
        simulates Depth Bounded Depth First Search

        if a path is found:
            Path will be printed
            self.found_goal = True will be set
        
        Assumptions:
            Edges weights are not considered

        Args:
            start: A Node object for the starting Node
            goal: A Node object for the goal Node
            depthBound: An integer value of the depth limit we are exploring

        Returns:
            count: An integer value of number of nodes in the constructed path
        """
        # init
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        depthQueue = [0]

        count = 0
        self.show_info('Starting DB-DFS')
        
        # kernel
        while queue:
            self.alg_iteration_start()

            # get the HEAD of OPEN
            node = queue.pop(0)
            visited.append(node)
            nodeDepth = depthQueue.pop(0)

            # check for GOAL node
            if node == goal:
                visited.append(node)
                self.found_goal=True
                break
            
            """
            If depth is under the bound:
                iterate over children
                if a child is not in OPEN or CLOSED:
                    set the current node as it's parent
                    insert the child into the OPEN
                    insert the child's depth in the depthQueue
            """
            if nodeDepth < depthBound:
                index = 0
                for neighbor in self.neighbors(node):
                    if neighbor not in visited and neighbor not in queue:
                        self.set_parent(neighbor,node)
                        queue.insert(index, neighbor)
                        depthQueue.insert(index, nodeDepth+1)
                        index = index + 1

                count = count + index
        
            self.alg_iteration_end()

        if self.found_goal:
            path=self.gen_path()
            self.show_path(path)
        else:
            self.show_info('No path available upto Depth %s' %depthBound)

        queue.clear()
        visited.clear()
        depthQueue.clear()

        return count