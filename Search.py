from heapq import heappop, heappush
class PriorityQueue:
    def __init__(self):
        self.data = []
    def push(self, item, cost):
        #self.data.append((cost, item))
        heappush(self.data, (cost,item) )
    def pop(self):
        #self.data.sort()
        return heappop(self.data)[1]
        #return self.data.pop(0)[1]
    def isEmpty(self):
        return len(self.data) == 0
    def __str__(self):
        return str([str(x[1]) for x in self.data])
class SearchNode:
    def __init__(self, state, parent, cost=0):
        self.state = state
        self.parent = parent
        self.cost = cost

    def path(self):
        if self.parent == None:
            return [self.state]
        else:
            return self.parent.path() + [self.state]
    def __str__(self):
        return "%s, %s" %(self.cost, self.state[0])

def ucSearch(successors, startState, goalTest, heuristic=lambda x: 0):
    if goalTest(startState):
        return [startState]
    startNode = SearchNode(startState, None, 0)
    agenda = PriorityQueue()
    agenda.push(startNode, heuristic(startState))
    expanded = set()
    while not agenda.isEmpty():
        parent = agenda.pop()
        if parent.state not in expanded:
            expanded.add(parent.state)
            if goalTest(parent.state):
                return parent.path(), len(expanded), parent.cost
            for childState, cost in successors(parent.state):
                child = SearchNode(childState, parent, parent.cost+cost)
                if childState in expanded:
                    continue
                agenda.push(child, child.cost+heuristic(childState))
    return None, len(expanded), -1


def search(successors, startState, goalTest, dfs = False):
    if goalTest(startState):
        return [startState]
    else:
        agenda = [SearchNode(startState, None)]
        visited = {startState}
        while len(agenda) > 0:
            if dfs: # Stack
                parent = agenda.pop(-1)
            else: # Queue
                parent = agenda.pop(0)
            for childState in successors(parent.state):
                child = SearchNode(childState, parent)
                if goalTest(childState):
                    return child.path()
                if childState not in visited:
                    agenda.append(child)
                    visited.add(childState)
        return None
