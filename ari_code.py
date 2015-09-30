# generate road map from problem
import networkx as nx
import matplotlib.pyplot as plt

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

def phase_one(problem):
    results = new_generateConstraintGraph(problem)
    # are we done?
    if results[0][0] == problem.num:
        print "Completed after phase 1. returning order and paths"
        return results[0][1], results[0][3]
    else:
        print "Phase 1 did not completely decouple problem. Starting phase 2"


def new_generateConstraintGraph(problem):
    # generate all paths
    paths = [None]*problem.num
    r_locs= zip(problem.startStates, problem.goalStates)
    total_c = 1
    for i, (s,g) in enumerate(r_locs):
        paths[i] = problem.rm.search(s,g)
        total_c *= len(paths[i])

    C = [(nx.DiGraph(), [])]
    
    for r, p_r in enumerate(paths):
        c_len = len(C)
        #print "c_len = %s,num paths = %s" % (c_len, len(p_r))
        # increase C by number of extra paths
        """
        tmp =[]
        for x in range(len(p_r)-1):
            C_ = [(c.copy(), list(e)) for (c,e) in C]
            tmp += C_
        C +=  tmp
        """
        #print "new c_len is %s " % len(C)
        all_constraints = []
        for i,p in enumerate(p_r):
            constraints = []
            # path = p 
            for robot, (s,g) in enumerate(r_locs):
                if robot == r: continue
                #print robot, s,g
                if s in p:
                    #print "%s goes before %s " %(robot, r)
                    constraints.append( (robot,r) )
                if g in p:
                    #print "%s goes after %s " %(robot, r)
                    constraints.append( (r, robot) )
            x = set(constraints)
            add = True
            swap = []
            for c,_ in all_constraints:
                if c.issubset(x) : add=False
                elif c.issuperset(x):
                    swap.append((c,_))
            if add:
                all_constraints.append((x,p))
            for c in swap:
                all_constraints.remove(c)


         # increase C by number of extra paths
       
        tmp =[]
        for x in range(len(all_constraints)-1):
            C_ = [(c.copy(), list(e)) for (c,e) in C]
            tmp += C_
        C +=  tmp
        for i, (constraints,p) in enumerate(all_constraints):
            for j in range(c_len):
                idx = i*c_len + j     
                C[idx][0].add_edges_from(constraints)
                C[idx][1].append(p)
           
    graphs = []
    for c,e in C:
        print nx.to_dict_of_lists(c)

        nx.draw_networkx(c)
        plt.plot()
        scc= list(nx.strongly_connected_components(c))
        scc.sort(reverse=True)
        graphs.append( (len(scc), scc, c, e))
    graphs.sort(reverse=True)
    return graphs

