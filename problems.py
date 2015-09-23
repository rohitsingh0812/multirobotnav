"""
figure 3b from paper.  Determine (n)umber of robots and degree of coupleing (alpha)
"""

def get_problem(n,alpha):
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



