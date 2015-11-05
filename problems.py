"""
    problems.py
    Author: Ariel Anders aanders@mit.edu

    This file has three different test problems.  a and b are from the paper.
    c generates a random problem.
"""

import random

"""
Robots are in a square.  Each moves to location +8 distances away
"""
def get_problem_a():
    starts = [(0,4), (1,4), (2,4), (3,4), (4,4), (4,3), (4,2), (4,1), (4,0), (3,0), (2,0), (1,0), (0,0), (0,1), (0,2), (0,3)]
    goals = [starts[(i+8)%16] for i in range(len(starts))]
    obstacles = [(2,2)]
    xMax,yMax = 5,5
    return xMax, yMax, starts, goals, obstacles


"""
figure 3b from paper.  
Determine (n)umber of robots and degree of coupleing (alpha)
"""
def get_problem_b(n,alpha):
    xMax = n/alpha + alpha -1
    yMax = alpha +2
    assert xMax > 0 and yMax > 0
    assert alpha > 1 and n/alpha > 0

    starts = []
    goals = []
    obstacles = []

    for x in range(n/alpha):
        for y in range(alpha):
            if y == 0:
               s = (x,alpha)
            else:
               s = (x, alpha - y - 1)
            g = (x,y)
            starts.append(s)
            goals.append(g)
    for x in range(0, xMax):
       if x == n/alpha -1: continue
       obs = (x, alpha+1)
       obstacles.append(obs)

    for x in range(n/alpha,xMax):
       for y in range(yMax):
           if y == alpha: continue
           obstacles.append((x,y))

    return xMax, yMax, starts, goals, obstacles
if __name__=="__main__":

    print get_problem(10,2)



"""
Random problem.  
Grid xMax,yMax with n robots.  Creates obstacles with 50% probabilty
"""
def get_problem_c(xMax=10,yMax=10,n=10):
    G = xMax*yMax
    k = G #2*n 
    starts = []
    goals = []
    obstacles = []
    sample = random.sample(xrange(G),k)

    for i,s in enumerate(sample):
        x = s % xMax
        y = int( s/xMax)

        if i < n:
            starts.append((x,y))
        elif i <2*n:
            goals.append((x,y))
        else:
            if random.random() < .1:
                obstacles.append((x,y))

    return xMax, yMax, starts, goals, obstacles

def get_problem_params(args):

	if args.p == 0:
		# generate row of objects
		xMax, yMax = 5,5
		obstacles=[[1,1], [2,2]]
		robotLocs = (0,0), (3,2), (4,3), (1,2), (1,4)#,(3,0)
		robotGoalLoc = (2,1), (3,4), (1,4), (0,0), (4,1)#, (0,1)
	elif args.p == 1:
		xMax,yMax = 3,3
		obstacles = [[1,0], [1,2]]
		robotLocs = (0,2), (0,1), (2,0)
		robotGoalLoc = (0,0), (2,1), (2,2)

	elif args.p == 2:
		xMax, yMax = 4,3
		obstacles=[[0,1], [0,2], [3,2], [3,1] , [1,1], [1,2]]
		robotLocs = (1,0),(3,0), (2,1), 
		robotGoalLoc = (2,0), (0,0), (2,2)
   
	elif args.p ==3:
		xMax,yMax = 4,3
		obstacles = [[1,1]]
		robotLocs = (0,1), (0,2), (3,2), (1,0)
		robotGoalLoc = (3,1), (3,0), (0,0), (1,2)
	elif args.p ==4:
		xMax,yMax = 20,20
		obstacles = [[i,i+2] for i in range(17)]
		robotLocs = [[3*i,i+1] for i in range(1)]
		robotGoalLoc = [[i+2,i+5] for i in range(1)]
	elif args.p == 5:
		xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_a()
	elif args.p ==6:
		xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_c()
        
        if args.n != None and args.a != None:
		xMax,yMax,robotLocs,robotGoalLoc,obstacles = get_problem_b(args.n,args.a)
		alpha = args.a
	else:
		alpha = None

	return xMax,yMax,obstacles,robotLocs,robotGoalLoc, alpha

