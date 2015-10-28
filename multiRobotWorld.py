"""
    multiRobotworld.py
    Author: Ariel Anders, aanders@mit.edu

    this program creates the worldmodel and defines the move primitives 
    for the robots. It has a successor function to use for planning
    with a single robot or multiple robots
"""

import time
import copy
import gridMap
from itertools import product 

dirs = ["north", "south", "east", "west"]
subdirs = ["north","south"]
names = "abcdefghijklmnpqrstuvwxyz"
actions = dirs +  ['nop']

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
	def __init__(self, xMax, yMax, obstacles, robotLocs, goalLocs):
		self.xMax = xMax
		self.yMax = yMax
		self.robotLocs = [list(rL) for rL in robotLocs]
		self.goalLocs = [list(rL) for rL in goalLocs]
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
		robot_colors =lambda x:  ['red', 'purple', 'blue', 'yellow', 'green'][x % 5]
		objects = [('', loc, 'black') for loc in self.obstacles]
		
		for i, goalLoc in enumerate(self.goalLocs):
			objects.append(('g%d' %i , goalLoc, 'grey'))
		
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


def successors(wm, single, extra_obstacles=[None], alpha=None):

	# only test if loc inside grid and not at static obstacle
	def legalMove(loc):
		if wm.legalLoc(loc):
			return not (wm.blockedByObst(loc) or tuple(loc) in extra_obstacles)
		else: return False

	def applyAction(action, robotLoc):
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

