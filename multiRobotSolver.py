"""
    multiRobotProblem.py
    Author: Ariel Anders, aanders@mit.edu

    Solver for the multi robot problem.

"""
import time
from argparse import ArgumentParser
from multiRobotWorld import WorldModel, successors, MovePrimitive, getMotionPrimitive, getPrimitives
from ari_solver import RoadMap, ari_solver
from problems import get_problem_params, get_problem_c
from rohit_solver import rohit_solver

	
manDist = lambda (x1,y1), (x2,y2): abs(x1-x2) + abs(y1-y2)
cartDist = lambda (x1,y1), (x2,y2): ((x1-x2)**2 + (y1-y2)**2)**.5


class Problem:
	def __init__(self, xMax,yMax, robotLocs, goalStates, obstacles, alpha, noh=False):
                self.xMax = xMax
                self.yMax = yMax
		self.robotLocs = robotLocs
		self.startStates =  tuple(tuple(rl) for rl in robotLocs) 
		self.goalStates = goalStates
		self.num = len(robotLocs)
		self.bots = range(self.num)
		self.obstacles = obstacles
		self.wm = WorldModel(xMax,yMax,obstacles,self.startStates,self.goalStates, display=False)
		self.noh = noh
                self.rm = RoadMap(self)
                self.alpha = alpha

	def getSearchParams(self,i):
		if type(i) == list:  #multiple robot
			start = tuple(self.startStates[r] for r in i)
			robotGoals = tuple(self.goalStates[r] for r in i)
			goalTest = lambda x:\
					all([x[j] == robotGoals[j] for j in range(len(i))])
			heuristic = lambda x:\
					sum([manDist(x[j],robotGoals[j]) for j in range(len(i))])
		else:
			goalTest = lambda x: x == self.goalStates[i]
			robotGoals = self.goalStates[i] 
			heuristic = lambda x: manDist(x, self.goalStates[i])
			start = self.startStates[i]
		if self.noh:
			print "no heuristc"
			heuristic = lambda x: 0
		return start, goalTest, heuristic, robotGoals

#XXX ROHIT HOW TO SEARCH :-) 
def individual_search(problem, source, target):
    return problem.rm.search(source,target)

def get_full_config_path(problem):
	# full config space plan:
	start,goalTest,heuristic,_ = problem.getSearchParams(problem.bots)
	path, expanded, Cost = ucSearch(successors(problem.wm,False, obstacles), start,goalTest,heuristic) 
	print "full config space plan. expanded nodes = %s , path length =%s "\
			% (expanded, len(path))
	return path

""" 
path is a dictionary is a list of tuples [ (r_idx, path_r_idx) ]
Each tuple has r_idx- index of the robot and path_r_idx that robots path.
The robot can be composite so the path must be executed simulatenously
"""
def ari_simulate_graphic(problem, path):
    problem.wm.draw()
    time.sleep(1)
    for (cr, path_r) in path:
        r_size = len(cr)
        if r_size == 1:
                r = cr[0]
                for p in range(1,len(path_r)):
                        prim = getMotionPrimitive(path_r[p-1], path_r[p])[0]
                        problem.wm.doi(r,prim)
                        time.sleep(1)
        else:
            path = path_r
            for p in range(1,len(path)):
                    prims = getPrimitives(path[p-1], path[p])
                    for z in range(r_size):
                            problem.wm.doi(cr[z], prims[z])
                    time.sleep(1)


def sketch_simulate_graphic(movelist, problem,filename, solved):
        if solved:
            with open(simulate_sol,'r') as f:
                lines = f.readlines()
                timedmoves = ast.literal_eval((lines[0]).strip())
        else:
            timedmoves = dict()
            for entry in movelist:
                    timestep = entry[0]
                    if timestep not in timedmoves:
                            timedmoves[timestep] = []
                    robot = entry[1]
                    move = entry[2]
                    mvstrlist = ["west","east","north","south","none"]
                    mvstr = mvstrlist[move]
                    if(mvstr!="none"):
                            timedmoves[timestep].append((robot,mvstr))
            with open(filename,'w') as f:
                f.write(str(timedmoves))
        
        for k in sorted(timedmoves.keys()):
                print "Time: " + str(k+1)
                for (robot,mvstr) in timedmoves[k]:
                        prim = MovePrimitive(mvstr)
                        problem.wm.doi(robot,prim)
                time.sleep(1)	



if __name__=="__main__":
	parser = ArgumentParser()
	
	parser.add_argument("--noh",action="store_true")
	parser.add_argument("--display", action="store_true")
	parser.add_argument("--p", type=int, default=0)
	
	parser.add_argument("--sketch",action="store_true")
	parser.add_argument("--z3",action="store_true")
	parser.add_argument("--waits_optimize",action="store_true")
	parser.add_argument("--simulate_sol",type=str,default="")
	parser.add_argument("--time_max", type=int, default=-1)
	parser.add_argument("--waits_reward", type=int, default=0)

	parser.add_argument("--n",type=int, default=None)
	parser.add_argument("--a",type=int, default=None)
	parser.add_argument("--full", action="store_true")

        parser.add_argument('--tests', action='store_true')

	
	args = parser.parse_args()

        
        if args.tests :
            results = ""
            for n in [5, 10]:
                alpha = None
                start = time.time()
                for i in range(10):
	            xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_c(15,15,n)
	            problem = Problem(xMax,yMax,robotLocs,robotGoalLoc,obstacles,alpha, args.noh)
                    start = time.time()
                    try:
                        movelist, name, solved, solve_t = rohit_solver(problem,args)
                        test_time  = time.time() - start    
                        results += "%d, %d, %.2f\n" %(n, solve_t, test_time)
                        with open('results.txt', 'wb') as f:
                            f.write(results)
                    except:
                        test_time  = time.time() - start    
                        results += "%d, -1, %.2f\n" %(n,  test_time)
                        with open('results.txt', 'wb') as f:
                            f.write(results)
                        print "failed!"
        else:
            xMax,yMax,obstacles,robotLocs,robotGoalLoc,alpha = get_problem_params(args)
	    problem = Problem(xMax,yMax,robotLocs,robotGoalLoc,obstacles,alpha, args.noh)
            problem.wm.draw()

            if args.simulate_sol != "" or args.sketch or args.z3:
                print "using rohit's solver!"
                movelist, name, solved = rohit_solver(problem,args)
                sketch_simulate_graphic(movelist,problem,name, solved)	

            else:
                print "using ari's solver!"
                path = ari_solver(problem,args)
                ari_simulate_graphic(problem, path)

