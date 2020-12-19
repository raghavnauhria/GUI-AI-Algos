class dfid(AlgorithmBase):

    def execute(self):
        # init
        count = -1
        previousCount = 0
        depthBound = 0
        start = self.start_nodes[0]
        goal = self.goal_nodes[0]

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
        # init
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        depthQueue = [0]

        count = 0
        self.show_info('Starting DB-DFS')
        
        # kernel
        while queue:
            self.alg_iteration_start()

            node = queue.pop(0)
            visited.append(node)
            nodeDepth = depthQueue.pop(0)

            # check for GOAL node
            if node == goal:
                visited.append(node)
                self.found_goal=True
                break
            
            # iterate over children
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