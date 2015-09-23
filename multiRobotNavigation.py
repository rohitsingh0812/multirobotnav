import pdb
import time
import copy
import sys
import math

from tarjan import tarjan
import gridMap
from itertools import product, chain
from collections import Counter
from  Search import ucSearch
from argparse import ArgumentParser
from genSketch import runSketch, findBestMoveList
import ast
from problems import get_problem_b, get_problem_a, get_problem_c

global alpha

class Primitive:
	def __init__(self):
		pass
	def __str__(self):
		return "Empty primitive"

class MovePrimitive(Primitive):
	def __init__(self, direction):
		Primitive.__init__(self)
		self.direction = direction
	def __str__(self):
		return self.direction

class WorldModel:
	def __init__(self, xMax, yMax, obstacles, robotLocs):
		self.xMax = xMax
		self.yMax = yMax
		self.robotLocs = [list(rL) for rL in robotLocs]
		self.home = [list(rL) for rL in robotLocs]
		self.obstacles = [list(o) for o in obstacles] # list of (x,y) pairs
		self.gridMap = gridMap.GridMap(xMax, yMax)

	
	def legalLoc(self, (x, y)):
		return x >= 0 and y >= 0 and x < self.xMax and y < self.yMax


	def blockedByObst(self, loc):
		return list(loc) in self.obstacles
	
	def moveLocByDirection(self, loc, direction):
		loc = copy.copy(loc)
		if direction == 'north':
			loc[1] += 1
		if direction == 'south':
			loc[1] -= 1
		if direction == 'east':
			loc[0] += 1
		if direction == 'west':
			loc[0] -= 1
		return loc

	def move(self, loc, direction):
		if direction == 'north':
			loc[1] += 1
		if direction == 'south':
			loc[1] -= 1
		if direction == 'east':
			loc[0] += 1
		if direction == 'west':
			loc[0] -= 1
		if direction != None: print 'PRIM: Moved', direction, 'to', loc
	
	def doi(self,i, prim, clear="noDraw"):
		if isinstance(prim, MovePrimitive):
				self.move(self.robotLocs[i], prim.direction)
		else:
				raise Exception, 'Unknown primitive' + str(prim)
		self.draw()
		return True

	def do(self, prims, clear = 'noDraw'):
		for i, prim in enumerate(prims):
			# Motion primitives
			if isinstance(prim, MovePrimitive):
				self.move(self.robotLocs[i], prim.direction)
			else:
				raise Exception, 'Unknown primitive' + str(prim)
		self.draw()
		return True

	def draw(self, color = 'cyan'):
		robot_colors =lambda x:  ['red', 'purple', 'blue'][x % 3]
		
		objects = [('', loc, 'black') for loc in self.obstacles]
		for i, robotLoc in enumerate(self.robotLocs):
			objects.append(('r%d' %i , robotLoc, robot_colors(i)))
		self.gridMap.drawWorld(objects)
		#XXXtime.sleep(1)

def moveLocByDirection(loc, direction):
		loc = copy.copy(loc)
		if direction == 'north':
			loc[1] += 1
		if direction == 'south':
			loc[1] -= 1
		if direction == 'east':
			loc[0] += 1
		if direction == 'west':
			loc[0] -= 1
		return loc



dirs = ["north", "south", "east", "west"]
subdirs = ["north","south"]
names = "abcdefghijklmnpqrstuvwxyz"
actions = dirs +  ['nop']

def successors(wm, single, extra_obstacles=[None]):

	# only test if loc inside grid and not at static obstacle
	def legalMove(loc):
		if wm.legalLoc(loc):
			return not (wm.blockedByObst(loc) or tuple(loc) in extra_obstacles)
		else: return False

	def applyAction(action, robotLoc):
                global alpha
                if alpha != None and robotLoc[1] != alpha:
                    valid_dirs = subdirs
                else:
                    valid_dirs = dirs

                if not action in valid_dirs:
                    return robotLoc
		if action =="nop":
			return robotLoc
		else:
			rl  = moveLocByDirection(list(robotLoc), action)
			if not legalMove(rl) :
				return None
			return tuple(rl)
			 

	def get_successors(robotStates):
		joint_actions = list(product(actions, repeat=len(wm.robotLocs)))
		bots = range(len(robotStates))
		next_states = []
		for joint_action in joint_actions:
			if all([act == "nop" for act in joint_action]):
				continue

			robotLocs = list(robotStates)
			
			for robot in bots:
				action = joint_action[robot]
				robotLoc = robotStates[robot]
				robotLocs[robot] = applyAction(action, robotLoc)
				if robotLocs[robot] == None: break

			if None in robotLocs: continue
			
			# check for robot in same location
			robot_duplicates = len(set(robotLocs)) < len(robotLocs)
			if robot_duplicates: continue
			
			swap = False
			for i in bots:
				for j in range(i+1,len(robotLocs)):
					if robotStates[i]==robotLocs[j]: 
						swap = True
						break
			if swap: continue
			
			nops = sum(["nop"==x for x in joint_action])
			cost = 1 + len(joint_action) - nops
			next_states.append( (tuple(robotLocs) , cost))
		return next_states
	def single_get_successors(robotLoc):
                global alpha
                if alpha!=None and  robotLoc[1] != alpha:
                    valid_dirs = subdirs
                else:
                    valid_dirs = dirs
		next_states = []
		for act in valid_dirs:
			rl = moveLocByDirection(list(robotLoc), act)
			if legalMove(rl):
				next_states.append((tuple(rl),1))
		return next_states
	if single:
		return single_get_successors
	else:
		return get_successors



def getPrimitives(state, next_state):
	prims = [getMotionPrimitive(state[i], next_state[i])[0] \
				for i in range(len(state))]
	return prims

def getMotionPrimitive( (x,y), (nx,ny)):
	move=None
	if ny - y == 1: move = "north"
	elif nx - x == 1: move = "east"
	elif ny - y == -1: move = "south"
	elif nx - x == -1: move ="west"
	return MovePrimitive(move), move

	
manDist = lambda (x1,y1), (x2,y2): abs(x1-x2) + abs(y1-y2)
cartDist = lambda (x1,y1), (x2,y2): ((x1-x2)**2 + (y1-y2)**2)**.5
 
def didFail():
		print "NO SOLUTION FOUND"
		sys.exit(1)

class Problem:
	def __init__(self, xMax,yMax, robotLocs, goalStates, obstacles, noh=False):
		self.robotLocs = robotLocs
		self.startStates =  tuple(tuple(rl) for rl in robotLocs) 
		self.goalStates = goalStates
		self.num = len(robotLocs)
		self.bots = range(self.num)
		self.obstacles = obstacles
		self.wm = WorldModel(xMax,yMax,obstacles,self.startStates)
		self.noh = noh
	
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
		#print "defining new search problem for robots %s. "\
		#		"Start states: %s, Goal states: %s" %  (i, start, robotGoals)
		return start, goalTest, heuristic, robotGoals

def generatePathandConstraints(problem):
	CR = []

	total_expanded = 0
	paths = [None]*problem.num
	for i in problem.bots:
		start,goalTest,heuristic,goal =problem.getSearchParams(i)
		paths[i], expanded, Cost = ucSearch( \
				successors(problem.wm, single=True), start,goalTest,heuristic)
		if paths[i] == None:
                        import pdb; pdb.set_trace()
			print "failed to find individual path,  no solution"
			didFail()

		total_expanded += expanded
		CR.append({'id':i, 'start':[start], 'goal':[goal], 'path':paths[i]})
	sortCR = generateConstraintGraph(CR)
	return sortCR, paths, total_expanded, CR

	
def genSubProblemObs(CR, i):
	obstacles = []
	for z in range(len(CR)):
		if z < i:
			pre_obs = [problem.goalStates[rl] for rl in CR[z]]
			obstacles += pre_obs
		elif z > i:
			post_obs =[problem.startStates[rl] for rl in CR[z]]
			obstacles += post_obs 
	return obstacles

def generateConstraintGraph(CR):
	C = {CR[i]['id']:[] for i in range(len(CR))}
	for i in range(len(CR)):
		s_i = CR[i]['start']
		g_i = CR[i]['goal']
		id_i = CR[i]['id']
		for j in range(len(CR)):
			if i == j: continue
			id_j = CR[j]['id']
			for s in s_i:
				if s in CR[j]['path'] or s in chain.from_iterable(CR[j]['path']):
					#print "%d should go before %d" % (id_i,id_j)
					if j not in C[id_i]:C[id_i].append(id_j) 
					continue
			for g in g_i:
				if g in CR[j]['path'] or g in chain.from_iterable(CR[j]['path']):
					#print "%d should go after  %d" % (id_i,id_j)
					if i not in C[id_j]:C[id_j].append(id_i)
					continue
	sequence = tarjan(C)
	sort_sequence = [sorted(sequence[i]) for i in range(len(sequence)-1,-1,-1)]
	return sort_sequence



def resolveConflict(problem, CR, i,C):
	# i is index of composite robot
	start,goalTest,heuristic,goals =problem.getSearchParams(CR[i])
	obstacles = genSubProblemObs(CR,i)
	# for each robot in CR[i] find single path
	paths = {}
	total_expanded = 0
	new_CR = []
	for j,r in enumerate(CR[i]):
		start,goalTest,heuristic,goal =problem.getSearchParams(r)
		# add other start and goal states as obstacles
		my_obs = []
		for k,other_r in enumerate(CR[i]):
			if k == j : continue
			start_k = problem.startStates[other_r]
			goal_k = problem.goalStates[other_r]
			if start != start_k and goal != start_k:
				my_obs.append(start_k)
			if start != goal_k and goal != goal_k:
				my_obs.append(goal_k)
		
		paths[r], expanded, Cost = ucSearch( successors(\
				problem.wm, True, obstacles+my_obs), start,goalTest,heuristic)
		total_expanded += expanded
		if paths[r] == None:
			paths[r], expanded, Cost = ucSearch( successors(\
					problem.wm, True, obstacles), start,goalTest,heuristic)
			total_expanded += expanded
		
		new_CR.append({'id':r,'start':[start], 'goal':[goal], 'path':paths[r]})
	new_sortCR = generateConstraintGraph(new_CR)
	new_CR = CR[:i] + new_sortCR + CR[i+1:]
	if len(new_CR) > len(CR):
		# use new CR!
		for r in new_sortCR:
			if len(r) ==1:
				C[r[0]]['path'] = paths[r[0]]

		#print "Updated  %s to %s " % (CR, new_CR)
		CR = new_CR
	
	#print "total expanded", total_expanded
	return CR, C, total_expanded

def get_problem_params(i=0):
	if i == 0:
		# generate row of objects
		xMax, yMax = 5,5
		obstacles=[[1,1], [2,2]]
		robotLocs = (0,0), (3,2), (4,3), (1,2), (1,4)#,(3,0)
		robotGoalLoc = (2,1), (3,4), (1,4), (0,0), (4,1)#, (0,1)
	elif i == 1:
		xMax,yMax = 3,3
		obstacles = [[1,0], [1,2]]
		robotLocs = (0,2), (0,1), (2,0)
		robotGoalLoc = (0,0), (2,1), (2,2)

	elif i == 2:
		xMax, yMax = 4,3
		obstacles=[[0,1], [0,2], [3,2], [3,1] , [1,1], [1,2]]
		robotLocs = (1,0),(3,0), (2,1), 
		robotGoalLoc = (2,0), (0,0), (2,2)
   
	elif i ==3:
		xMax,yMax = 4,3
		obstacles = [[1,1]]
		robotLocs = (0,1), (0,2), (3,2), (1,0)
		robotGoalLoc = (3,1), (3,0), (0,0), (1,2)
	elif i ==4:
		xMax,yMax = 20,20
		obstacles = [[i,i+2] for i in range(17)]
		robotLocs = [[3*i,i+1] for i in range(1)]
		robotGoalLoc = [[i+2,i+5] for i in range(1)]

        elif i ==5:
                xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem(6,2)
                global alpha
                alpha = yMax -1
        elif i == 6:
            xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_a()
        elif i ==7:
            xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_c()

	return xMax,yMax,obstacles,robotLocs,robotGoalLoc
 
def get_full_config_path(problem):
	# full config space plan:
	start,goalTest,heuristic,_ = problem.getSearchParams(problem.bots)
	path, expanded, Cost = ucSearch(successors(problem.wm,False, obstacles), start,goalTest,heuristic) 
	print "full config space plan. expanded nodes = %s , path length =%s "\
			% (expanded, len(path))
	return path



if __name__=="__main__":
        global alpha
	parser = ArgumentParser()
	
	parser.add_argument("--noh",action="store_true")
	parser.add_argument("--display", action="store_true")
	parser.add_argument("--p", type=int, default=0)
	
	parser.add_argument("--sketch",action="store_true")
	parser.add_argument("--simulate_sol",type=str,default="")
	parser.add_argument("--time_max", type=int, default=-1)
	parser.add_argument("--waits_reward", type=int, default=0)

        parser.add_argument("--n",type=int, default=None)
        parser.add_argument("--a",type=int, default=None)
        parser.add_argument("--full", action="store_true")

	
	args = parser.parse_args()
	xMax,yMax,obstacles,robotLocs,robotGoalLoc = get_problem_params(args.p)
        
        if args.n != None and args.a != None:
            xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem(args.n,args.a)
            alpha = args.a
        else:
            alpha = None
       	
        problem = Problem(xMax,yMax,robotLocs,robotGoalLoc,obstacles,args.noh)
        if args.full:
	    get_full_config_path(problem)
	problem.wm.draw()
        raw_input("Done")
	if args.simulate_sol != "":
		with open(args.simulate_sol,'r') as f:
			lines = f.readlines()
			timedmoves = ast.literal_eval((lines[0]).strip())
			for k in sorted(timedmoves.keys()):
				print "Time: " + str(k+1)
				for (robot,mvstr) in timedmoves[k]:
					prim = MovePrimitive(mvstr)
					problem.wm.doi(robot,prim)
				time.sleep(1)
	elif args.sketch:
		if(args.time_max > 0):
			movelist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,args.time_max,args.waits_reward)
		else:
			movelist = findBestMoveList(xMax,yMax,obstacles,robotLocs,robotGoalLoc)
		
		if(len(movelist) == 0):
			print "Couldn't find solution with Sketch, TODO: change TMAX etc"
			exit(1)
		else:
			#each entry in movelist is (time,robotID, move)
			#move 0 - LEFT/west, 1- RIGHT/east, 2- UP/north, 3 - DOWN/south, 4- NO CHANGE
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
			for k in sorted(timedmoves.keys()):
				print "Time: " + str(k+1)
				for (robot,mvstr) in timedmoves[k]:
					prim = MovePrimitive(mvstr)
					problem.wm.doi(robot,prim)
				time.sleep(1)
			with open('temp_'+str(args.p)+'.sol','w') as f:
				f.write(str(timedmoves))
				
	else:
		
		#if args.p != 0:
	        #		get_full_config_path(problem)

		compositeRobots,paths, total_expanded,C= generatePathandConstraints(problem)
		CR = list(compositeRobots)
		print CR
		print "Phase 1: %s \t expanded: %d" %(CR, total_expanded)
		
		for i,cr in enumerate(compositeRobots):
			if len(cr) > 1: # composite robot
				CR, C, expanded = resolveConflict(problem, CR, i, C)
				total_expanded += expanded
		
		print "Phase 2: %s \t expanded: %d" %(CR, total_expanded)
		
		# Attempt to find path for coupled robots within their sub problems
		for i,cr in enumerate(CR):
			if len(cr) > 1: 
				start,goalTest,heuristic,goals =problem.getSearchParams(cr)
				obstacles = genSubProblemObs(CR,i)
				path, expanded, Cost = ucSearch(successors(\
						problem.wm,False, obstacles), start,goalTest,heuristic)
				total_expanded += expanded
				if path == None: 
					C[cr[0]]['path'] = None
				C[cr[0]]['path'] = path  # store path in robot of lowest index
				for j in range (1, len(cr)):
					C[cr[j]]['path'] = None # all other paths are none
		
		print "Phase 3. Solve for composite robots.  \t expanded: %d" %(total_expanded)
	  
		totalSteps = sum([len(C[cr[0]]['path']) for cr in CR ])
		print "Solution found. Total expanded nodes is %s , total time steps in execution = %s"\
				% (total_expanded, totalSteps)
			
		if args.display:
			for i,cr in enumerate(CR):
				r_size = len(cr)
				if r_size == 1:
					r = cr[0]
					path_r = C[r]['path']
					for p in range(1,len(path_r)):
						prim = getMotionPrimitive(path_r[p-1], path_r[p])[0]
						problem.wm.doi(r,prim)
						time.sleep(1)
				else:
					path = C[cr[0]]['path']
					totalSteps += len(path)
					for p in range(1,len(path)):
						prims = getPrimitives(path[p-1], path[p])
						for z in range(r_size):
							problem.wm.doi(cr[z], prims[z])
						time.sleep(1)

  
        raw_input("Done")
