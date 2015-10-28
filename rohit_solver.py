"""
    Rohits solver!
"""
from genSketch import runSketch, findBestMoveList, findBestMoveListLinear, runz3
import ast

def rohit_solver(problem, args):
    
    if args.simulate_sol != "":
            movelist = None
            solved = True
            name = args.simulate_sol
    elif args.sketch or args.z3:
            if args.time_max > 0 and args.sketch:
                    movelist = runSketch('temp.sk',problem,args.time_max,args.waits_reward,problem.alpha)
            elif args.time_max > 0 and args.z3:
                    movelist = runz3(problem,args.time_max,0,problem.alpha)
            elif args.sketch:
                    movelist = findBestMoveList(problem,problem.alpha,args.waits_optimize)
            else:
                    print "Invalid options for --sketch or --z3"
                    exit(1)
                    
            if(len(movelist) == 0):
                    print "Couldn't find solution with Sketch/z3, TODO: change TMAX etc"
                    exit(1)
            else:
                    #each entry in movelist is (time,robotID, move)
                    #move 0 - LEFT/west, 1- RIGHT/east, 2- UP/north, 3 - DOWN/south, 4- NO CHANGE
                    fname = 'temp'
                    if(args.p > 0 ):
                            fname = fname+"_p" + str(args.p)
                    if(args.a):
                            fname = fname + "_a" + str(args.a)
                    if(args.n):
                            fname = fname + "_n" + str(args.n)
                    name = fname + '.sol'
                    solved = False

    return movelist, name, solved
