import sys
from numpy import *
from heapq import *
import cv2

def userSetStart(res):
    start_x = float(input('x position of the start point (0, 249): '))
    start_y = float(input('y position of the start point (0, 149): '))
    start = (int(start_y*res), int(start_x*res))
    if start_x < 0 or start_x >= 250 or start_y < 0 or start_y >= 150:
        raise Exception("Invalid start point! A valid point should be on a 150x250 map")
    return start

def userSetGoal(res):
    goal_x = float(input('x position of the goal point (0, 249): '))
    goal_y = float(input('y position of the goal point (0, 149): '))
    goal = (int(goal_y*res), int(goal_x*res))
    if goal_x < 0 or goal_x >= 250 or goal_y < 0 or goal_y >= 150:
        raise Exception("Invalid goal point! A valid point should be on a 150x250 map")
    return goal

'''Obstacle space'''
res = float(input('Resolution / Grid Size for the map: '))
mymap = ones((int(150*res),int(250*res), 3))*255
#  rectangle
cv2.rectangle(mymap,(round(50*res),round(67.5*res)),(round(100*res),round(112.5*res)),(0,0,0),-1)
#  ellipse
cv2.ellipse(mymap,(round(140*res),round(120*res)),(round(15*res),round(6*res)),0,0,360,(0,0,0),-1)
#  circle
cv2.circle(mymap, (round(190*res),round(130*res)),round(15*res), (0,0,0), -1)
#  polygum
pts = array([[125,56],[163,52],[170,90],[193,52],[173,15],[150,15]], int32)
pts = rint((pts.reshape((-1,1,2))*res)).astype(int32)
cv2.fillPoly(mymap,[pts],(0,0,0))
#'''

'''Augmented obstacle space'''
radius = input('The radius or the robot: ')
clearance = input('The clearance or obstables: ')
clear = (float(radius)+float(clearance))
r = round(clear*res)+1
kernel = zeros((2*r-1,2*r-1),uint8)
for i in range(2*r-1):
    for j in range(2*r-1):
        if (r-1-i)**2 + (r-1-j)**2 < r**2:
            kernel[i,j] = 1

dilation = cv2.erode(mymap,kernel,iterations = 1)
dilation[where((dilation == [0,0,0]).all(axis = 2))] = [0,0,255]
dilation[where((mymap==[0,0,0] ).all(axis = 2))] = [0,0,0]
mymap = dilation


'''set up the goal and the start point'''
goal = userSetGoal(res)  # set goal point
start = userSetStart(res)

if array_equal(mymap[goal], [0,0,0]) or array_equal(mymap[goal], [0,0,255]):
    print('Warning: The goal is not reseachable!')
    print('         It is within the (augmented) obstacle space')
else: mymap[goal[0], goal[1], :] = [0,255,255]

if array_equal(mymap[start], [0,0,0]) or array_equal(mymap[start], [0,0,255]):
    raise Exception('The robot is trapped within the (augmented) obstacle space')
else: mymap[start[0], start[1], :] = [255,0,0]

'''initailize the cost map'''
infinity = 10000
costMap = ones(mymap[...,0].shape)*infinity
costMap[start] = 0

openlist = [] 
heappush(openlist, (0, start))

parent = {start:None}
move_tran = [(1,0), (0,1), (-1, 0), (0, -1)]
move_diag = [(1,1), (-1,1), (1,-1), (-1, -1)]

cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.resizeWindow("map", 1000, 600)

count = 0
while openlist:
    _, curr = heappop(openlist)
    for move in move_tran+move_diag:
        node = tuple(map(sum,zip(move,curr)))
        if 0<=node[0]<len(mymap) and 0<=node[1]<len(mymap[0]) \
           and not array_equal(mymap[node], [0,0,255])        \
           and not array_equal(mymap[node], [0,0,0]):
            # calculate new cost
            cost_to_come = costMap[curr] + min(1.4, abs(move[0])+abs(move[1]))
            if node not in parent:
                parent[node] = curr
                heappush(openlist, (cost_to_come, node))
                costMap[node] = cost_to_come
                mymap[node[0], node[1], :] = [127,0,0]
                count += 1
            elif costMap[node] > cost_to_come:
                parent[node] = curr
                costMap[node] = cost_to_come

    mymap[curr[0], curr[1], :] = [255,255,0]
    cv2.imshow('map', cv2.flip(mymap, 0))  # show image
    cv2.waitKey(1)

    if curr == goal:
        node = goal
        while node:
            mymap[node[0], node[1], :] = [0,255,255]
            node = parent[node]
            cv2.imshow('map', cv2.flip(mymap, 0))  # show image
            cv2.waitKey(1)
        print("The goal is found!")
        print("Searched area is: ", count)
        print("The toal cost is: ", costMap[goal])
        break

if goal not in parent:
    print("The goal is NOT found!")

cv2.waitKey(0)
cv2.destroyAllWindows()
