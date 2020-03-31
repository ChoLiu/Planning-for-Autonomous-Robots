from numpy import *
from heapq import *
import cv2


#def heuristic(node, goal):
#    return ((node[0]-goal[0])**2 + (node[1]-goal[1])**2)**0.5


def setNode(y, x, res=0.2):
    node = (int(y*res), int(x*res))
    if x < 0 or x >= 250 or y < 0 or y >= 150:
        raise Exception("A valid point should be within a 150x250 map")
    return node


def generateMap(res=0.5):
    '''Obstacle space'''
    #res = 0.2
    mymap = ones((int(150*res), int(250*res), 3))*255
    #  rectangle
    ## GAP 15
    cv2.rectangle(mymap,
                  (round(120*res), round(70*res)),
                  (round(160*res), round(85*res)),
                  (0, 0, 0), -1)
    ## GAP 20
    cv2.rectangle(mymap,
                  (round(120*res), round(40*res)),
                  (round(160*res), round(50*res)),
                  (0, 0, 0), -1)

    cv2.rectangle(mymap,
                  (round(120*res), round(20*res)),
                  (round(160*res), round(10*res)),
                  (0, 0, 0), -1)
    ## GAP 30
    cv2.rectangle(mymap,
                  (round(120*res), round(130*res)),
                  (round(160*res), round(149*res)),
                  (0, 0, 0), -1)
    #  ellipse
    cv2.ellipse(mymap,
                (round(40*res), round(50*res)),
                (round(15*res), round(6*res)),
                0, 0, 360, (0, 0, 0), -1)
    #  circle
    cv2.circle(mymap,
               (round(220*res), round(80*res)),
               round(15*res), (0, 0, 0), -1)
    cv2.circle(mymap,
               (round(50*res), round(120*res)),
               round(15*res), (0, 0, 0), -1)

    cv2.circle(mymap,
               (round(206*res), round(40*res)),
               round(10*res), (0, 0, 0), -1)
    #  polygum
    # pts = array([[125, 56],
    #              [163, 52],
    #              [170, 90],
    #              [193, 52],
    #              [173, 15],
    #              [150, 15]], int32)
    # pts = rint((pts.reshape((-1, 1, 2))*res)).astype(int32)
    # cv2.fillPoly(mymap, [pts], (0, 0, 0))
    return mymap

def circleKernel(r):
    kernel = zeros((2*r-1, 2*r-1), uint8)
    for i in range(2*r-1):
        for j in range(2*r-1):
            if (r-1-i)**2 + (r-1-j)**2 < r**2:
                kernel[i, j] = 1
    return kernel

def augmentMap(mymap, res=0.5, clear=5):
    '''Augmented obstacle space'''
    clear = 5
    r = round(clear*res) + 1
    kernel = circleKernel(r)

    dilation = cv2.erode(mymap, kernel, iterations=1)
    dilation[where((dilation == [0, 0, 0]).all(axis=2))] = [0, 0, 255]
    dilation[where((mymap == [0, 0, 0]).all(axis=2))] = [0, 0, 0]
    return dilation


def randomConfigurasion(mymap, graph):
    y = random.choice(mymap.shape[0])
    x = random.choice(mymap.shape[1])

    while array_equal(mymap[(y, x)], [0, 0, 0]) or \
          array_equal(mymap[(y, x)], [0, 0, 255]) or \
          (y, x) in graph:
        y = random.choice(mymap.shape[0])
        x = random.choice(mymap.shape[1])
    
    return (y, x)


def nearestVertex(graph, node):
    if not graph:
        print("The graph is empty!")
        return None

    # near = (0, 0)
    near_dis = 1000000
    for curr in graph:
        dis = (curr[0] - node[0]) ** 2 + (curr[1] - node[1]) ** 2
        if dis < near_dis:
            near_dis = dis
            near = curr
    
    return near


def testReachability(mymap, vertex, direction, stepSize, var):
    newNode = vertex

    for step in range(1, int(stepSize) + 1):
        ratio = step / (direction[0]**2 + direction[1]**2)**0.5
        diff = (round(direction[0] * ratio), round(direction[1] * ratio))
        tmp = (vertex[0] + diff[0], vertex[1] + diff[1])
        # print("tmp: ", tmp)

        if not (0 <= tmp[0] < mymap.shape[0] and 0 <= tmp[1] < mymap.shape[1]):
            continue

        r = int((var+step)/30)
        # print("kernel:\n", r)
        for i in range(2*r+1):
            for j in range(2*r+1):
                if (i-r)**2 + (j-r)**2 > r**2:
                    continue
                pos = (tmp[0] + i - r, tmp[1] + j - r)
                # print("pos", pos)
                if not (0 <= pos[0] < mymap.shape[0] and 0 <= pos[1] < mymap.shape[1]):
                    return newNode, step
                if array_equal(mymap[pos], [0, 0, 0]) or array_equal(mymap[pos], [0, 0, 255]):
                    return newNode, step
        newNode = tmp
        
    return newNode, stepSize



def newConfigurasion(mymap, vertex, node, stepSize, var):
    direction = (node[0] - vertex[0], node[1] - vertex[1])

    return testReachability(mymap, vertex, direction, stepSize, var)


def rrtPlanning(mymap, goal, start, stepSize):

    # Check the validility of the goal and the start
    if array_equal(mymap[goal], [0, 0, 0]):
        print('Warning: The goal is not reseachable!')
        print('         It is within obstacle space')
    elif array_equal(mymap[goal], [0, 0, 255]):
        print('Warning: The goal is not reseachable!')
        print('         It is within obstacle space')
    else:
        mymap[goal[0], goal[1], :] = [0, 255, 255]

    if array_equal(mymap[start], [0, 0, 0]):
        raise Exception('The robot is trapped within obstacle space')
    elif array_equal(mymap[start], [0, 0, 255]):
        raise Exception('The robot is trapped within obstacle space')
    else:
        mymap[start[0], start[1], :] = [255, 0, 0]

    cv2.circle(mymap, (start[1],start[0]), 1, [0, 255, 0])
    cv2.circle(mymap, (goal[1],goal[0]), 1, [0, 255, 0])
    parent = {}
    parent[start] = None
    drift = {}
    drift[start] = 0

    for i in range(50000):
        rand = randomConfigurasion(mymap, parent)
        near = nearestVertex(parent, rand)
        newNode, movement = newConfigurasion(mymap_, near, rand, stepSize, drift[near])
        if newNode in parent:
            continue

        parent[newNode] = near
        drift[newNode] = drift[near] + movement
        #print("parent", parent)
        # Draw edge
        cv2.line(mymap, (newNode[1], newNode[0]), (near[1], near[0]), [255, 0, 0])
        # mymap[rand] = [0, 255, 0]
        cv2.imshow('map', cv2.flip(mymap, 0))
        cv2.waitKey(1)

        if (newNode[0] - goal[0])**2 + (newNode[1] - goal[1])**2 <= 10**2:
            print("Found the goal!")
            succ = newNode
            pred = parent[succ]
            while pred:
                cv2.line(mymap, (succ[1], succ[0]), (pred[1], pred[0]), [255, 255, 0])
                cv2.circle(mymap, (succ[1], succ[0]), int(drift[succ]/30), [0, 255, 0])
                succ = pred
                pred = parent[pred]
                # print(drift[succ])
                cv2.imshow('map', cv2.flip(mymap, 0))
                cv2.waitKey(1)
            return
       


    print("Cannot find the goal!")
    return



if __name__ == "__main__":


    step_size = 8
    # Set the map
    res_ = 1
    clear_ = 1
    mymap_ = generateMap(res=res_)
    mymap_ = augmentMap(mymap_, res=res_, clear=clear_)

    # set up the goal and the start point
    goal_ = setNode(10, 210, res_)
    start_ = setNode(0, 0, res_)

    cv2.namedWindow("map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("map", 1000, 600)

    #print('goal: ', goal_)
    #print('goal: ', start_)
    #randomConfigurasion(mymap_)
    graph = [start_, goal_]
    # print(nearestVertex(graph, (15, 31)))
    stepSize_ = 5
    rrtPlanning(mymap_, goal_, start_, stepSize_)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

