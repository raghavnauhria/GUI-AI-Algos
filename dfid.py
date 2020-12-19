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
            count = dbdfs(start, goal, depthBound)
            depthBound = depthBound + 1

        if self.found_goal:
            path=self.gen_path()
            self.show_path(path)
        else:
            self.show_info('No path available')

    
    def dbdfs(self, start, goal, depthBound):
        # init
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        count = 0
        depth = 1
        
        # kernel
        while queue:
            self.alg_iteration_start()

            node = queue.pop(0)
            visited.append(node)

            # check for GOAL node
            if node == goal:
                visited.append(node)
                self.found_goal=True
                break
            
            # iterate over children
            if depth < depthBound:
                index = 0
                for neighbor in self.neighbors(node):
                    if neighbor not in visited and neighbor not in queue:
                        self.set_parent(neighbor,node)
                        queue.insert(index, neighbor)
                        index = index + 1

                depth = depth + 1
                count = count + index
        
            self.alg_iteration_end()

        queue.clear()
        visited.clear()
        return count