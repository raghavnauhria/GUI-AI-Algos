class dfs(AlgorithmBase):
    
    def execute(self):
        # init
        start=self.start_nodes[0]
        goal=self.goal_nodes[0]
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        
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
            index = 0
            for neighbor in self.neighbors(node):
                if neighbor not in visited and neighbor not in queue:
                    self.set_parent(neighbor,node)
                    queue.insert(index, neighbor)
                    index = index + 1
        
            self.alg_iteration_end()
            
        if self.found_goal:
            path=self.gen_path()
            self.show_path(path)
        else: self.show_info('No path available')
