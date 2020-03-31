from numpy import *
from heapq import *
import cv2

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
   
    cv2.rectangle(mymap,
                  (round(120*res), round(70*res)),
                  (round(160*res), round(85*res)),
                  (0, 0, 0), -1)
   
    cv2.rectangle(mymap,
                  (round(120*res), round(40*res)),
                  (round(160*res), round(50*res)),
                  (0, 0, 0), -1)

    cv2.rectangle(mymap,
                  (round(120*res), round(20*res)),
                  (round(160*res), round(10*res)),
                  (0, 0, 0), -1)
    cv2.rectangle(mymap,
                  (round(120*res), round(130*res)),
                  (round(160*res), round(149*res)),
                  (0, 0, 0), -1)
    

    cv2.rectangle(mymap,
                  (round(25*res), round(130*res)),
                  (round(45*res), round(145*res)),
                  (0, 0, 0), -1)
    cv2.rectangle(mymap,
                  (round(25*res), round(90*res)),
                  (round(45*res), round(110*res)),
                  (0, 0, 0), -1)
  
    cv2.rectangle(mymap,
                  (round(25*res), round(50*res)),
                  (round(45*res), round(70*res)),
                  (0, 0, 0), -1)

    cv2.rectangle(mymap,
                  (round(25*res), round(30*res)),
                  (round(45*res), round(10*res)),
                  (0, 0, 0), -1)
    return mymap


def circleKernel(r):
    kernel = zeros((2*r-1, 2*r-1), uint8)
    for i in range(2*r-1):
        for j in range(2*r-1):
            if (r-1-i)**2 + (r-1-j)**2 < r**2:
                kernel[i, j] = 1
    return kernel


def augmentMap(mymap, res=0.2, clear=5):
    '''Augmented obstacle space'''
    clear = 5
    r = round(clear*res) +3
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

    near = (33, 5)
    near_dis = (near[0] - node[0]) ** 2 + (near[1] - node[1]) ** 2
    for curr in graph:
        dis = (curr[0] - node[0]) ** 2 + (curr[1] - node[1]) ** 2
        if dis < near_dis:
            near_dis = dis
            near = curr
    
    return near

def nextPos(dt, ul, ur):
    r = 3.8
    L = 23.0
    k1 = ((ul + ur) * r) / 2
    k2 = ((ur - ul) * r) / L
    dth = k2 * dt
    if ul != ur:
        dx = (k1 / k2) * (sin(k2 * dt) - sin(0))
        dy = -(k1 / k2) * (cos(k2 * dt) - cos(0))
    else:
        dx = k1 * dt
        dy = 0

    return dx, dy, dth


def rotation(the, x, y):
    new_x = cos(the) * x - sin(the) * y
    new_y = sin(the) * x + cos(the) * y
    return new_x, new_y


def testReachability(mymap, vertex, action, stepSize, var, the):
    pts = [vertex]
    pre_dis = 0
    pre_dth = 0

    for step in range(1, int(stepSize) + 1):

        dx, dy, dth = nextPos(step, action[0], action[1])
        dx, dy = rotation(the, dx, dy)

        tmp = (vertex[0] + int(round(dy)), vertex[1] + int(round(dx)))
        #print(tmp5

        if not (0 <= tmp[0] < mymap.shape[0] and 0 <= tmp[1] < mymap.shape[1]):
            continue

        r = int((var + step)/30) # Uncertainty is propotional to the traveled distance
        
        for i in range(2*r+1):
            for j in range(2*r+1):
                if (i-r)**2 + (j-r)**2 > r**2:
                    continue

                pos = (tmp[0]+i-r, tmp[1]+j-r)
                # print("pos", pos)
                if not (0 <= pos[0] < mymap.shape[0] and 0 <= pos[1] < mymap.shape[1]):
                    return pts, step, pre_dth
                if array_equal(mymap[pos], [0, 0, 0]) or array_equal(mymap[pos], [0, 0, 255]):
                    return pts, step, pre_dth
        pts.append(tmp)
        pre_dth = dth
        
    return pts, stepSize, dth



def newConfiguration(mymap, vertex, node, stepSize, var, rpm, orientation):
    actions = [(0, rpm[1]), (rpm[1], 0), (rpm[1], rpm[1]), (rpm[0], rpm[1]), (rpm[1], rpm[0])]
    pts_opt = [vertex]
    drift_opt = 0
    dth_opt = 0
    dis = 1000**2
    

    for action in actions:
        pts, drift, dth = testReachability(mymap, vertex, action, stepSize, var, orientation[vertex])
        if (pts[-1][0] - node[0])**2 + (pts[-1][1] - node[1])**2 < dis:
            
            dis = (pts[-1][0] - node[0])**2 + (pts[-1][1] - node[1])**2
            pts_opt = pts
            drift_opt = drift
            dth_opt = dth
        
    return pts_opt, drift_opt, dth_opt


def rotatedRect(mymap, node, the, color=(0, 50, 10)):
    pts = array([rotation(the, -4, -3),
                 rotation(the, -4, +3),
                 rotation(the, +4, +3),
                 rotation(the, +4, -3)])
    pts = [[node[1] + pt[0], node[0] + pt[1]] for pt in pts]
    pts = array(pts).astype(int32)
    # cv2.polylines(mymap, [pts],True, (0, 0, 0),-1)
    cv2.fillPoly(mymap, [pts], color)
    cv2.polylines(mymap, [pts], True, (0, 0, 0))

    # cv2.fillPoly(mymap, [pts], (0, 0, 0))


def rrtPlanning_nonholo(mymap, goal, start, stepSize, rpm):

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


    parent = {}
    parent[start] = None
    drift = {}
    drift[start] = 0
    orientation = {}
    orientation[start] = 0
    # rotatedRect(mymap, start, orientation[start], (255, 0, 0))
    # rotatedRect(mymap, goal, 0, (0, 255, 0))
    cv2.imshow('map', cv2.flip(mymap, 0))
   

    for i in range(50000):
        rand = randomConfigurasion(mymap, parent)
        near = nearestVertex(parent, rand)
        pts, uncertain, dth = newConfiguration(mymap, near, rand, stepSize,
                                                drift[near], rpm, orientation)

        newNode = pts[-1]
        if newNode in parent:
            continue

        parent[newNode] = near
        drift[newNode] = drift[near] + uncertain
        orientation[newNode] = orientation[near] + dth
        # Draw edge
        pre = near
        for pos in pts[1:]:
            cv2.line(mymap, (pre[1], pre[0]), (pos[1], pos[0]), [255, 0, 0])
            pre = pos
        #cv2.polylines(mymap, [array([[pt[1], pt[0]] for pt in pts])], True, (255, 0, 0))
        cv2.imshow('map', cv2.flip(mymap, 0))
        cv2.waitKey(1)

        if (newNode[0] - goal[0])**2 <= 100 and (newNode[1] - goal[1])**2 <= 100 :
            print("Found the goal!")
            succ = newNode
            pred = parent[succ]
            count = 0
            while pred:
                cv2.line(mymap, (succ[1], succ[0]), (pred[1], pred[0]), [255, 0, 155])
                cv2.circle(mymap, (succ[1], succ[0]), int(drift[succ]/30), [0, 255, 0])
                # if count %3 == 0:
                #     rotatedRect(mymap, succ, orientation[succ])
                succ = pred
                pred = parent[pred]
                # print(drift[succ])
                count += 1
                cv2.imshow('map', cv2.flip(mymap, 0))
                cv2.waitKey(1)
            return
       

    print("Cannot find the goal!")
    return

if __name__ == "__main__":

    # Set the map
    res_ = 1
    clear_ = 5
    mymap_ = generateMap(res=res_)
    mymap_ = augmentMap(mymap_, res=res_, clear=clear_)

    # set up the goal and the start point
    goal_ = setNode(140, 230, res_) # Top goal
    # goal_ = setNode(73, 110, res_)
    start_ = setNode(33, 5, res_)

    # RRT's configuration
    step_size = 5
    rpm_ = (1, 0.5)

    cv2.namedWindow("map", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("map", 1000, 600)

    graph = [start_, goal_]
    # print(nearestVertex(graph, (15, 31)))
    stepSize_ = 3
    rrtPlanning_nonholo(mymap_, goal_, start_, stepSize_, rpm_)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
