from subprocess import call,  Popen, PIPE, check_output, CalledProcessError, STDOUT
import ast
import time
import math

from ari_solver import get_paths

def writeSketch(filename,problem,TMAX,waitsMIN,alpha):
	xMax = problem.wm.xMax
	yMax = problem.wm.yMax
	obstacles = problem.obstacles 
	robotLocs = problem.robotLocs
	robotGoalLoc = problem.goalStates
	f = open(filename,'w\n')
	NUMBOTS= problem.num
	assert NUMBOTS == len(robotGoalLoc);
	
	#take required parameters and hardcode everything!
	f.write('#define LEFT 0\n')
	f.write('#define RIGHT 1\n')
	f.write('#define UP 2\n')
	f.write('#define DOWN 3\n')
	f.write('#define WAIT 4\n')
	f.write('#define TMAX '+str(TMAX)+'\n')
	f.write('#define waitsMIN '+str(waitsMIN)+'\n')
	f.write('#define xMax '+str(xMax)+' \n')
	f.write('#define yMax '+str(yMax)+' \n')
	f.write('#define NUMBOTS '+str(NUMBOTS)+' \n')
	f.write('\n')
	f.write('\n')
	f.write('void MOVING(int time, int robot, int move);\n')
	f.write('void WAITREWARD(int waits);\n')
	f.write('\n')
	f.write('generator void moveOrWait(ref int curx, ref int cury, int i, int t, ref int waits){\n')
	f.write('	int move = ??(3);\n')
	if(alpha != None):
		#only allowed moves would be UP or DOWN or WAIT unless cury==alpha
		f.write('	if(cury != '+str(alpha)+'){\n')
		f.write('		assert(move != LEFT && move != RIGHT);\n')
		f.write('	}')
	f.write('		assert(move == LEFT || move == RIGHT || move == UP || move ==DOWN || move == WAIT);\n')
	f.write('	\n')
	f.write('	if(move == LEFT){\n')
	f.write('		curx = curx -1;\n')
	f.write('		cury = cury;		\n')
	f.write('	}\n')
	f.write('	else if(move == RIGHT){\n')
	f.write('		curx = curx +1;\n')
	f.write('		cury = cury;\n')
	f.write('	}\n')
	f.write('	else if(move == UP){\n')
	f.write('		curx = curx;\n')
	f.write('		cury = cury +1;\n')
	f.write('	}\n')
	f.write('	else if(move == DOWN){\n')
	f.write('		curx = curx;\n')
	f.write('		cury = cury -1;\n')
	f.write('	}\n')
	f.write('	MOVING(t,i,move);\n')
	f.write('	waits = waits + (t+1)*(move==WAIT);\n')
	f.write('	assert(curx < xMax && curx >= 0 && cury < yMax && cury >= 0);\n')
	
	for p in obstacles:
		x=p[0]
		y=p[1]
		f.write('	assert(curx != '+str(x)+' || cury != '+str(y)+');\n')	
	
	f.write('}\n')
	f.write('\n')
	f.write('\n')
	f.write('\n')
	f.write('harness void main(){\n')
	f.write('	\n')
	for i in range(NUMBOTS):
		f.write('		int locsx_'+str(i)+'='+str(robotLocs[i][0])+' ; \n')
		f.write('		int locsy_'+str(i)+'='+str(robotLocs[i][1])+' ; \n')
		f.write('		int goalsx_'+str(i)+'='+str(robotGoalLoc[i][0])+' ; \n')
		f.write('		int goalsy_'+str(i)+'='+str(robotGoalLoc[i][1])+' ; \n')
	f.write('\n')
	#f.write('		int t=0;\n')
	f.write('		int waits =0;\n')
	for t in range(TMAX):
	
		for  k in range(NUMBOTS):
			f.write('		moveOrWait(locsx_'+str(k)+', locsy_'+str(k)+', '+str(k)+', '+str(t)+',waits);\n')

			f.write('		//this guy cannot collide with anyone else\n')
			for i in range(NUMBOTS):
				if(i!=k):
					f.write('		assert (locsx_'+str(i)+' != locsx_'+str(k)+' || locsy_'+str(i)+' != locsy_'+str(k)+');\n')
		f.write('		\n')
		#f.write('		t++;\n')
	#f.write('		assert (t <= TMAX);\n')
	f.write('		assert( waits >= waitsMIN);\n')
	f.write('		WAITREWARD( waits );\n')
	for l in range(NUMBOTS):
		f.write('		assert(goalsx_'+str(l)+' == locsx_'+str(l)+' && goalsy_'+str(l)+' == locsy_'+str(l)+');\n')
	f.write('}\n')

	f.close()
	
def runSketch(filename,problem,TMAX,waitsMIN,alpha):
	writeSketch(filename,problem,TMAX,waitsMIN,alpha)
	#call(["sk9l",filename])
	try:
		X = check_output(['/usr/bin/sk9l', filename],stderr=STDOUT)
	except CalledProcessError, e:
		return []
	#output, err = p.communicate()
	#rc = p.returncode
	if "DONE" not in X:
		print "Not solved"
		return []
	movelist = []
	waitval = -1
	for line in X.split('\n'):
		if ('MOVING' in line) and ('uninterp' not in line):
			line = line.replace('MOVING(','[')
			line = line.replace(');',']')
			movelist.append(ast.literal_eval(line.strip()))
		if ('WAITREWARD' in line) and ('uninterp' not in line):
			line = line.replace('WAITREWARD(','')
			line = line.replace(');','')
			waitval =int(line)
			
			#print line
	return movelist
	


def binary_search(minval,maxval,criterion):
	#find smallest value that works, we know that maxval works
	working_state = None
	working_val = -1
	while(True):
		currval = int((minval+maxval)/2)
		(worked,state) = criterion(currval)
		if(worked):
			working_state = state
			working_val = currval
			maxval = currval
		else:
			minval = currval
		if(maxval - minval <= 1):
			return (working_val,working_state)

			
			

def findBestMoveList(problem,alpha,do_waits):
	#Doing binary search for appropriate values of TMAX and waitsMIN
	waitsMIN = 0
	def crit_tmax(val):
		print "TRYING TMAX,waitsMIN: ", val, waitsMIN
		temp_mvlist = runSketch('temp.sk',problem,val,waitsMIN,alpha)
		return (len(temp_mvlist) > 0, temp_mvlist)
	
	TMAX_max = (problem.wm.xMax+problem.wm.yMax)*2+problem.num
	
	TMAX, movelist = binary_search(1,TMAX_max,crit_tmax)
	
	if(do_waits):
		def crit_waitsmin(val):
			print "TRYING TMAX,waitsMIN: ", TMAX, -val
			temp_mvlist = runSketch('temp.sk',problem,TMAX,-val,alpha)
			return (len(temp_mvlist) > 0, temp_mvlist)	
		waitsMIN_max = (TMAX*problem.num*(TMAX+1))/2
		waitsMIN, movelist = binary_search(-waitsMIN_max,0,crit_waitsmin)
		waitsMIN = -waitsMIN	
	
	print "Optimal TMAX,waitsMIN: ",TMAX,waitsMIN
	return movelist
	
	
def findBestMoveListLinear(problem,alpha):
	#Doing binary search for appropriate values of TMAX and waitsMIN
	waitsMIN=0
	#first find best TMAX
	TMAX_min = 1
	TMAX_max = min(100,(problem.wm.xMax+problem.wm.yMax)*2+problem.num)
	TMAX_curr = 1
	while(TMAX_curr < TMAX_max):#works on max and doesn't work on min
		print "CURR TMAX: ",TMAX_curr
		temp_mvlist = runSketch('temp.sk',problem,TMAX_curr,waitsMIN,alpha)
		if(len(temp_mvlist) > 0):
			#found it!
			movelist = temp_mvlist
			TMAX = TMAX_curr
			break
		else:
			TMAX_curr = TMAX_curr + 1		
				
	print "Optimal TMAX = " +str(TMAX)
	return movelist
	
	
from z3 import *

def addBitsToz3(NUMBOTS,timeStep,bits,s):
	for i in range(NUMBOTS):
		bits[i][timeStep] = dict() 
		for DIR in ['l','r','u','d']:
			bits[i][timeStep][DIR] = Bool('r'+str(i)+'_t'+str(timeStep)+'_'+DIR)
		#constraints for only one of them being true or none
		constr = And(Not(bits[i][timeStep]['l']),Not(bits[i][timeStep]['r']),Not(bits[i][timeStep]['u']),Not(bits[i][timeStep]['d']))
		constr = Or(constr,And(bits[i][timeStep]['l'],Not(bits[i][timeStep]['r']),Not(bits[i][timeStep]['u']),Not(bits[i][timeStep]['d'])))
		constr = Or(constr,And(Not(bits[i][timeStep]['l']),bits[i][timeStep]['r'],Not(bits[i][timeStep]['u']),Not(bits[i][timeStep]['d'])))
		constr = Or(constr,And(Not(bits[i][timeStep]['l']),Not(bits[i][timeStep]['r']),bits[i][timeStep]['u'],Not(bits[i][timeStep]['d'])))
		constr = Or(constr,And(Not(bits[i][timeStep]['l']),Not(bits[i][timeStep]['r']),Not(bits[i][timeStep]['u']),bits[i][timeStep]['d']))
		s.add(constr)

def addConstraintsIsValid(xMax,yMax,obstacles,s,curpos):
	s.add(curpos[0] < xMax)
	s.add(curpos[0] >= 0)
	s.add(curpos[1] < yMax)
	s.add(curpos[1] >= 0)
	for (ox,oy) in obstacles:
		s.add(Or(ox != curpos[0], oy != curpos[1]))

def getLocs(robotLocs,curpos):
	i=0
	for (rx,ry) in robotLocs:
		curpos[i] = (rx,ry)
		i = i+1
import copy
def verifyMoveList(problem,movelist):
	#each entry in movelist is (time,robotID, move)
	#move 0 - LEFT/west, 1- RIGHT/east, 2- UP/north, 3 - DOWN/south, 4- NO CHANGE
	movelist.sort(key=lambda x:x[0])
	
	xMax = problem.wm.xMax
	yMax = problem.wm.yMax
	
	obstacles = problem.obstacles 
	rlocs = problem.robotLocs
	robotGoalLoc = problem.goalStates
	NUMBOTS= problem.num
	assert NUMBOTS == len(robotGoalLoc);
	robotLocs = []
	for (a,b) in rlocs:
		robotLocs.append((a,b))
	for (t,r,mv) in movelist:
		curx = robotLocs[r][0]
		cury = robotLocs[r][1]
		#print r,": (",curx,cury,")"
		newx = curx
		newy = cury
		if mv == 0: #LEFT
			newx = curx - 1
		elif mv == 1:
			newx = curx + 1
		elif mv == 2:
			newy = cury + 1
		elif mv == 3:
			newy = cury - 1
		#print r,": (",newx,newy,")"
		robotLocs[r] = (newx,newy)
	for r in range(NUMBOTS):
		if robotLocs[r][0] != robotGoalLoc[r][0] or robotLocs[r][1] != robotGoalLoc[r][1] :
			print "failed here: ",robotLocs
			print robotGoalLoc
			exit(1)
			
		
def boop(path,s,curpos, deviation):
        constraints = []
        for (x,y) in path:
            constraints.append( And(
                    curpos[0] <= x+ deviation, 
                    curpos[0] >= x-deviation,
                    curpos[1] <= y+ deviation, 
                    curpos[1] >= y-deviation,
                ) )
        big_constraint = Or(* constraints)
        s.add(big_constraint)

def runz3(problem,TMAX,waitsMIN,alpha):	
	xMax = problem.wm.xMax
	yMax = problem.wm.yMax
	
	obstacles = problem.obstacles 
	robotLocs = problem.robotLocs
	robotGoalLoc = problem.goalStates
	NUMBOTS= problem.num
	assert NUMBOTS == len(robotGoalLoc);
	
	#boolean variables to find
	#r_RNUM_t_TNUM_DIR 
	s=Solver()
	bits = dict()
	for i in range(NUMBOTS):
		bits[i] = dict()
	
	#for j in range(TMAX):
	#	addBitsToz3(NUMBOTS,j,bits,s)
	
	curpos = dict()
	getLocs(robotLocs,curpos)
	print curpos
	goalpos = dict()
	getLocs(robotGoalLoc,goalpos)
	print goalpos
	nextpos = dict()


        deviation = 3
        paths = get_paths(problem)
	
	TMAX_max = (problem.wm.xMax+problem.wm.yMax)*2+problem.num
        for j in range(TMAX_max):
		addBitsToz3(NUMBOTS,j,bits,s)
		#move according to bits defined above

		for i in range(NUMBOTS):
			if(alpha != None):
				#only allowed moves would be UP or DOWN or WAIT unless cury==alpha
				s.add(Implies(curpos[i][1] != alpha,And(Not(bits[i][j]['l']),Not(bits[i][j]['r']))))
			nextpos[i] = (None,None)
			nextpos[i] = (If(bits[i][j]['l'],curpos[i][0]-1,
						 		If(bits[i][j]['r'],curpos[i][0]+1, 
						 			curpos[i][0]
						 		)
						 	),
						  If(bits[i][j]['d'],curpos[i][1]-1,
						 		If(bits[i][j]['u'],curpos[i][1]+1, 
						 			curpos[i][1]
						 		)
						 	)
						 )	 	
			addConstraintsIsValid(xMax,yMax,obstacles,s,nextpos[i])	
                        """ add ari constraints to shortest path"""
                        boop(paths[i],s,nextpos[i], deviation)

		#Constraints to make sure they don't collide AND swapping of places doesn't happen!
		#(1) nextpos[i] != nextpos[k] for any k < i 
		#(2) nextpos[i] != curpos[k] || nextpos[k] != curpos[i] for all k != i 
		for i in range(NUMBOTS):
			for k in range(i):
				s.add(Or(nextpos[i][0] != nextpos[k][0],nextpos[i][1] != nextpos[k][1]))
				s.add(Or(Or(nextpos[i][0] != curpos[k][0],nextpos[i][1] != curpos[k][1]),
						 Or(nextpos[i][0] != curpos[k][0],nextpos[i][1] != curpos[k][1])))
				s.add(Or(Or(curpos[i][0] != nextpos[k][0],curpos[i][1] != nextpos[k][1]),
						 Or(curpos[i][0] != nextpos[k][0],curpos[i][1] != nextpos[k][1])))
		for i in range(NUMBOTS):
			curpos[i] = nextpos[i]
			
		s.push()
		#goal assertions
		for r in range(NUMBOTS):
			s.add(curpos[r][0] == goalpos[r][0])
			s.add(curpos[r][1] == goalpos[r][1])			
		if( s.check() == sat):
			#TODO: add fixed paths as assumed bits to be true! can encode paths here!
			print "Solved with t = " + str(j+1)
			#print s
			M = s.model()
			#print M
			#each entry in movelist is (time,robotID, move)
			#move 0 - LEFT/west, 1- RIGHT/east, 2- UP/north, 3 - DOWN/south, 4- NO CHANGE
			movelist = []
			for t in range(j+1):
				for r in range(NUMBOTS):
					if is_true(M[bits[r][t]['l']]):
						#print t,r,"XAXAXAXA"
						movelist.append((t,r,0))
					elif is_true(M[bits[r][t]['r']]):
						movelist.append((t,r,1))
					elif is_true(M[bits[r][t]['u']]):
						movelist.append((t,r,2))
					elif is_true(M[bits[r][t]['d']]):
						movelist.append((t,r,3))
					else:
						movelist.append((t,r,4))
			#print movelist
			verifyMoveList(problem,movelist)
			return movelist, j
		else:
			s.pop()
			if(j == TMAX-1):
				print s
			print "Not solvable with t = " + str(j+1)
	
	return [], -1	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
