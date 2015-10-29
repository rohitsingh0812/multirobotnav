# generate road map from problem
import networkx as nx
import matplotlib.pyplot as plt
import time
import sys

import operator, functools
from Search import ucSearch
from multiRobotWorld import successors
from itertools import product, permutations


def ari_solver(problem,args):
   return phase_one(problem)

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
    results = iterative_generateConstraintGraph(problem)
    # are we done?
    solution = None
    if results[0] == problem.num:
        print "Completed after phase 1. returning order and paths"
        num_comp, scc, c, paths = results
        scc = [list(sc) for sc in scc]
        print sc, paths
        solution = [( sc, paths[sc[0]] ) for sc in scc]

    else:
        print "Phase 1 did not completely decouple problem"
        solution= phase_two(problem,results)
        if solution == None:
            solution = phase_three(problem, results)

    if solution== None:
        print "could not solve problem!"
    else:
        print "solved problem, solution is:"
        print solution
    return solution

def phase_two(problem, results):
    print "Phase 2 solving SCC sub problems"
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
                            problem.wm,False, obstacles, problem.alpha), start,goalTest,heuristic)
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


def minimum_constraints(problem, r, paths):
    constraints = [constraints_for_single_path(problem, p, r) for p in paths]
    is_minterm = [True]*len(paths)
    for i,c in enumerate(constraints):
        for j in range(i+1, len(constraints)):
            c_ = constraints[j]
            if c < c_ : is_minterm[j] = False
            if c > c_: is_minterm[i] = False
            if c == c_: is_minterm[j] = False
    return [(constraints[i], paths[i]) for i in range(len(paths)) if is_minterm[i]]

  


def phase_three(problem, results):
    print "Phase 3 solving SCC full problems"
    num_comp, scc, c, paths = results
    solution = [None]*len(scc)

    for i,sc in enumerate(scc):
        sc = list(sc)
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
                c.add_node(sc[0], name=sc) # only add first node back
                # get constraints
                constraints = constraints_for_path(problem, path,sc)
                print constraints
                c.add_edges_from(constraints)
                paths[sc[0]] = path


    num_comp, scc, max_sc = getSCC(c)
    solution = [None]*len(scc)

    for i,sc in enumerate(scc):
        sc = list(sc)
        if 'name' in c.node[sc[0]]:
            solution[i] = (c.node[sc[0]]['name'], paths[sc[0]])
        else:
            solution[i] = (sc, paths[sc[0]])

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
    paths = [problem.rm.search(s,g) for i, (s,g) in enumerate(r_locs)]
    mid = time.time()
    print "collecting all paths took : %s" % (mid-start)
    all_constraints = {r:minimum_constraints(problem,r,p_r) for r,p_r in enumerate(paths)}
    c_len = functools.reduce(operator.mul, [len(all_constraints[a]) for a in all_constraints])
    print "num constraint graphs", c_len
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
        scc_len, scc, max_scc  = getSCC(c)
        x = tuple(scc)
        if x in all_scc:
            continue
        else:
            all_scc.append(x)
            #print "duplicated scc"
        graphs.append( (scc_len, scc, c, e))
    graphs.sort(reverse=True)
    finish = time.time()
    print "middle to finish: %s, total :%s " % (finish - mid, finish-start)
    return graphs

def iterative_generateConstraintGraph(problem):
    start = time.time()
    # generate all paths
    paths = [None]*problem.num
    r_locs= zip(problem.startStates, problem.goalStates)
    paths = [problem.rm.search(s,g) for i, (s,g) in enumerate(r_locs)]
    mid = time.time()
    print "collecting all paths took : %s" % (mid-start)
    all_constraints = {r:minimum_constraints(problem,r,p_r) for r,p_r in enumerate(paths)}
    c_len = functools.reduce(operator.mul, [len(all_constraints[a]) for a in all_constraints])
    print "num constraint graphs", c_len

    vals = [range(len(all_constraints[r])) for r in all_constraints]
    ordering = list(product(*vals))
    best_max_scc = None
    for order in ordering:
        G = nx.DiGraph()
        G.add_nodes_from(problem.bots)
        for r, c in enumerate(order):
            G.add_edges_from(all_constraints[r][c][0])
        scc_len, scc, max_scc  = getSCC(G)
        if max_scc == 1:
            p = [ all_constraints[r][c][1] for (r,c) in enumerate(order)]
            return (scc_len, scc, G, p)
        
        if best_max_scc == None or max_scc < best_max_scc:
            best_max_scc = max_scc
            best_scc = scc
            best_p = [ all_constraints[r][c][1] for (r,c) in enumerate(order)]
            best_G = G
            best_scc_len = scc_len


    print "no solution founds"
    return (best_scc_len, best_scc, best_G, best_p)

