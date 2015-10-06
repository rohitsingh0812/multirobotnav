# generate road map from problem
import networkx as nx
import matplotlib.pyplot as plt
import time
import sys

import operator, functools
from Search import ucSearch

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
            ret = list(nx.all_shortest_paths(self.G, source, target))
                
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


def constraints_for_path(problem, path, cr):
    e = cr[0]
    constraints = set()

    for r in problem.bots:
        if r in cr: continue
        s = problem.startStates[r]
        g = problem.goalStates[r]
        for p in path:
            if s in p:
                constraints.add( (e,r) )
            if g in p:
                constraints.add( (r,e) )
    return constraints


def phase_three(problem, results):
    print "Phase 3 solving SCC full problems"
    from multiRobotNavigation import successors
    num_comp, scc, c, paths = results
    solution = [None]*len(scc)

    for i,sc in enumerate(scc):
        sc = list(sc)
        if len(sc) ==1:
                solution[i] = (sc, paths[sc[0]])

        if len(sc) >1 :
            if len(sc) >= 3:
                print "sc is too long.  won't attempt to find path"
                return None
            print "trying to find composite path for %s " % sc
            start,goalTest,heuristic,goals =problem.getSearchParams(sc)
            path, expanded, Cost = ucSearch(successors(\
				problem.wm,False), start,goalTest,heuristic)
            if path == None:
                print "no solution found"
                return None

            else:
                print "Path found. need to reorder constraint graph!"
                for n in sc: c.remove_node(n) # remove composite nodes
                c.add_node(sc[0]) # only add first node back
                # get constraints
                constraints = constraints_for_path(problem, path,sc)
                c.add_edges_from(constraints)
                solution[i] = (sc, path)

    num_comp, scc, max_sc = getSCC(c)
    print scc
    if max_sc == 1:
        print "solved problem!"
        return solution
    else:
        print "composite planner did not decrease size of SCC to 1"
        return None
                
def new_generateConstraintGraph(problem):
    start = time.time()
    # generate all paths
    paths = [None]*problem.num
    r_locs= zip(problem.startStates, problem.goalStates)
    total_c = 1
    for i, (s,g) in enumerate(r_locs):
        if not nx.has_path(problem.rm.G,s,g):
            print "FAILED TO FIND INDIVIDUAL PATH"
        paths[i] = problem.rm.search(s,g)
        total_c *= len(paths[i])
        #print "search complete"
    #print "finished individual path search"
    mid = time.time()
    print "collecting all paths took : %s" % (mid-start)
    G = nx.DiGraph()
    G.add_nodes_from(problem.bots)
    C = [(G, [])]
    all_constraints = {}
    
    for r, p_r in enumerate(paths):
        all_constraints[r] = []
        for i,p in enumerate(p_r):
            constraints = constraints_for_single_path(problem, p,r)
            add = True
            swap = []
            #XXX JUST DO THIS AT END
            for c,_ in all_constraints[r]:
                if c.issubset(constraints) : add=False
                elif c.issuperset(constraints):
                    swap.append((c,_))
            if add:
                all_constraints[r].append((constraints,p))
            for c in swap:
                all_constraints[r].remove(c)
    
    c_len = functools.reduce(operator.mul, [len(all_constraints[a]) for a in all_constraints])
    C = [None]*c_len
    for i in range(c_len):
        G = nx.DiGraph()
        G.add_nodes_from(problem.bots)
        C[i] = (G, [])
    
    for r in all_constraints:
        num_c = len(all_constraints[r])
        copies =c_len/num_c
        for i in range(num_c):
            constraints,p = all_constraints[r][i]
            for j in range(copies):
                idx = i*copies + j
                C[idx][0].add_edges_from(constraints)
                C[idx][1].append(p)

    graphs = []
    all_scc = []
    for c,e in C:
        #print nx.to_dict_of_lists(c)
        #nx.draw_networkx(c)
        #plt.plot()
        
        scc= list(nx.strongly_connected_components(c))
        scc.sort(reverse=True)
        x = tuple(scc)
        if x in all_scc:
            continue
        else:
            all_scc.append(x)
            #print "duplicated scc"
        print scc
        graphs.append( (len(scc), scc, c, e))
    graphs.sort(reverse=True)
    finish = time.time()
    print "middle to finish: %s, total :%s " % (finish - mid, finish-start)
    return graphs

