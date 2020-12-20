import numpy as np

class dfid(AlgorithmBase):

    def execute(self):
        # init
        # count = -1
        # previousCount = 0
        depthBound = 0
        cheapestUnexpanded = self.heuristic(start, goal)    # f-value of start node

        start = self.start_nodes[0]
        goal = self.goal_nodes[0]

        while (self.found_goal == False):
            # previousCount = count
            depthBound = cheapestUnexpanded
            self.show_info('For depth bound: %s' %depthBound)
            cheapestUnexpanded = self.dbdfs(start, goal, depthBound)

        if self.found_goal:
            self.show_info('Path found at (cost) Depth %s' % depthBound)
        else:
            self.show_info('No path available')

    
    def dbdfs(self, start, goal, depthBound):
        # init
        queue,visited=self.get_list('open'),self.get_list('closed')
        queue.append(start)
        
        g={i:np.inf  for i in self.get_nodes()}
        g[start]=0
        f={start:self.heuristic(start,goal)}
        
        nextDepth = depthBound
        count = 0
        
        self.show_info('Starting DB-DFS')
        
        # kernel
        while True:
            self.alg_iteration_start()

            # find first node in OPEN with f-value under the bound
            nodeIndex = next((index for index, node in enumerate(queue) if f[node] <= depthBound), -1)
            
            # if not found, update f-val and break
            if nodeIndex == -1:
                nextDepth = min(queue, key=lambda x: f[x])
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
                    f[neighbor] = g[neighbor] + self.heuristic(neighbor,goal)

                    # add to OPEN
                    queue.insert(index, neighbor)
                    index = index + 1

            self.alg_iteration_end()

        if self.found_goal:
            path=self.gen_path()
            self.show_path(path)
        else:
            self.show_info('No path available upto Depth %s' %depthBound)

        queue.clear()
        visited.clear()
        return nextDepth
                        