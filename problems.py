"""
figure 3b from paper.  Determine (n)umber of robots and degree of coupleing (alpha)
"""
import random
def get_problem_a():
    starts = [(0,4), (1,4), (2,4), (3,4), (4,4), (4,3), (4,2), (4,1), (4,0), (3,0), (2,0), (1,0), (0,0), (0,1), (0,2), (0,3)]
    goals = [starts[(i+8)%16] for i in range(len(starts))]
    obstacles = [(2,2)]
    xMax,yMax = 5,5
    return xMax, yMax, starts, goals, obstacles

def get_problem_c(xMax=10,yMax=10,n=10,o=10):
    G = xMax*yMax
    k = 2*n + o
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
            obstacles.append((x,y))

    return xMax, yMax, starts, goals, obstacles


    
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



