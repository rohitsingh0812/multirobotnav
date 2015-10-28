# generate road map from problem
import networkx as nx
import matplotlib.pyplot as plt
import time
import sys

import operator, functools
from Search import ucSearch
def ari_solver(problem,args):
    print "not implemented!"
    sys.exit(1)


class RoadMap:
    def __init__(self, problem):
        self.G = nx.Graph()
        
        xCheck = lambda x: x  >= 0 and x < problem.xMax
        yCheck = lambda x: x  >= 0 and x < problem.yMax
        check = lambda (x,y): xCheck(x) and yCheck(y) 
        dirs = [ (0,1), (0,-1), (1,0), (-1,0) ]

        for x in range(problem.xMax):
            for y in range(problem.yMax):
                if not (x,y) in self.G:
                    self.G.add_node( (x,y) )
                edges = [(x+i,y+j) for i,j in dirs]
                for e in edges:
                    if check(e):self.G.add_edge( (x,y), e)
        obs = [tuple(o) for o in problem.obstacles]
        self.G.remove_nodes_from(obs)
    
    def search(self,source,target):
        try:
            #XXX https://networkx.github.io/documentation/latest/reference/algorithms.html
            ret = list(nx.shortest_path(self.G, source, target))
                
        except:
                ret = None
        return ret

def getSCC(c):
    scc= list(nx.strongly_connected_components(c))
    scc.sort(reverse=True)
    max_sc = max([len(sc) for sc in scc])
    return len(scc), scc, max_sc

def genSubProblemObs(problem, CR, i):
	obstacles = []
	for z in range(len(CR)):
		if z < i:
			pre_obs = [problem.goalStates[rl] for rl in CR[z]]
			obstacles += pre_obs
		elif z > i:
			post_obs =[problem.startStates[rl] for rl in CR[z]]
			obstacles += post_obs 
	return obstacles


def phase_one(problem):
    results = new_generateConstraintGraph(problem)
    # are we done?
    solution = None
    if results[0][0] == problem.num:
        print "Completed after phase 1. returning order and paths"
        num_comp, scc, c, paths = results[0]
        solution =  zip(scc, paths)
    else:
        print "Phase 1 did not completely decouple problem"
        for res in results:
            solution= phase_two(problem,res)
            if solution == None:
                solution = phase_three(problem, res)
            if solution != None:
                break

    if solution== None:
        print "could not solve problem!"
    else:
        print "solved problem, solution is:"
        print solution


def phase_two(problem, results):
    print "Phase 2 solving SCC sub problems"
    from multiRobotNavigation import successors
    solved = True
    num_comp, scc, c, paths = results
    solution = [None]*len(scc)

    for i,sc in enumerate(scc):
        sc = list(sc)
        
        if len(sc) ==1:
            solution[i] = (sc, paths[sc[0]])
        
        if len(sc) > 1:
            if len(sc) >= 3:
                print "sc is too long.  won't attempt to find path"
                solved = False
                break

            print "trying to resolve sub problem for %s "% sc 
            start,goalTest,heuristic,goals =problem.getSearchParams(sc)
            obstacles = genSubProblemObs(problem, scc, i)
            print start, goals, obstacles
            path, expanded, Cost = ucSearch(successors(\
                            problem.wm,False, obstacles), start,goalTest,heuristic)
            if path == None:
                print "failed to solve sub problem."
                solved = False
                break
            else:
                solution[i] = (sc, path)

    if not solved:
        return None
    else:
        return solution



def constraints_for_single_path(problem, path, i):
    constraints = set()
    for r in problem.bots:
        if r ==i : continue
        s = problem.startStates[r]
        g = problem.goalStates[r]
        if s in path:
                constraints.add( (i,r) )
        if g in path:
                constraints.add( (r,i) )
    return constraints
