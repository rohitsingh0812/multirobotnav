from subprocess import call,  Popen, PIPE, check_output, CalledProcessError, STDOUT
import ast
import time
import math

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
	for line in X.split('\n'):
		if ('MOVING' in line) and ('uninterp' not in line):
			line = line.replace('MOVING(','[')
			line = line.replace(');',']')
			movelist.append(ast.literal_eval(line.strip()))
			#print line
	return movelist
	

#runSketch('temp.sk',5,5,[[1,1], [2,2]],((0,0), (3,2), (4,3), (1,2), (1,4)),((2,1), (3,4), (1,4), (0,0), (4,1)), 6, 58,3)
	

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
	#movelist = runSketch('temp.sk',problem,6,58,alpha)
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
	#movelist = runSketch('temp.sk',problem,6,58,alpha)
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
	
	
	
	
	
	
	
	
	
