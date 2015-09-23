from subprocess import call,  Popen, PIPE, check_output, CalledProcessError, STDOUT
import ast
import time
import math

def writeSketch(filename,xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN,nbitsT):
	f = open(filename,'w\n')
	NUMBOTS= len(robotLocs)
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
	f.write('	assert(move == LEFT || move == RIGHT || move == UP || move ==DOWN || move == WAIT);\n')
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
	
def runSketch(filename,xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN):
	nbitsT = int(math.log(TMAX,2) + 1)
	writeSketch(filename,xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN,nbitsT)
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
	
	
def findBestMoveList(xMax,yMax,obstacles,robotLocs,robotGoalLoc):
	#movelist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,6,58)
	#Doing binary search for appropriate values of TMAX and waitsMIN
	waitsMIN=0
	
	
	#first find best TMAX
	TMAX_min = 1
	TMAX_max = (xMax+yMax)*2+len(robotLocs)
	while(True):#works on max and doesn't work on min
		TMAX_curr = int((TMAX_max + TMAX_min)/2) 
		print "CURR TMAX: ",TMAX_min, TMAX_max
		temp_mvlist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX_curr,waitsMIN)
		if(len(temp_mvlist) > 0):
			#go left
			TMAX_max = TMAX_curr
		else:
			#go right
			TMAX_min = TMAX_curr		
				
		if TMAX_max - TMAX_min <= 1:
			TMAX=TMAX_max
			break
	print "Optimal TMAX = " +str(TMAX)
	#movelist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN)
	#time.sleep(2)
	
	#then find best waitsMIN
	waitsMIN_min = 0
	waitsMIN_max = (TMAX*len(robotLocs)*(TMAX+1))/2
	while(True):#works on min and doesn't work on max
		waitsMIN_curr = int((waitsMIN_max + waitsMIN_min)/2)
		print "CURR waitsMIN: ",waitsMIN_min, waitsMIN_max
		temp_mvlist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN_curr)
		if(len(temp_mvlist) == 0):
			#go left
			waitsMIN_max = waitsMIN_curr
		else:
			#go right
			waitsMIN_min = waitsMIN_curr
		if waitsMIN_max - waitsMIN_min <= 1:
			waitsMIN=waitsMIN_min
			break;
	print "Optimal waitsMIN = " +str(waitsMIN)
	movelist = runSketch('temp.sk',xMax,yMax,obstacles,robotLocs,robotGoalLoc,TMAX,waitsMIN)
	time.sleep(2)
	return movelist
	
	
	
	
	
	
	
	
	
	
