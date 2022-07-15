from math import atan2, sqrt, cos, sin
from tkinter import *

px,py,x,y = 0,0,0,0
lineIndex = 0
currRotation = 0
PI = 3.141592

class coordinate:

    def __init__(self, x, y):
        self.getX = x
        self.getY = y 

def distToPoint(cx, cy, px, py):
  return sqrt( (px-cx)**2 + (py-cy)**2 ) 

def minError(target,current):
    b = max(target,current)
    s = min(target,current)
    diff = b - s
    if diff <= 180:
        return(diff)
    else:
        return(360-b + s)

def dtr(input):
  return(PI*input/180)

def rtd( input):
  return(input*180/PI)

def sign(input):
    return(-1 if input < 0 else 1) 

def dirToSpin(target,currHeading):
  # -1 = clockwise
  # 1 = counterclockwise
  # currHeading = unscaled if unscaled < 180 else unscaled - 360
  diff = target - currHeading;
  if(diff < 0):
      diff += 360
  if(diff > 180):
      return 1
  else:
      return -1

def absoluteAngleToPoint( px,  py):
   global x,y
   try: 
    t = atan2(px-x,py-y)
   except:
    t = PI/2
   return t * (180/PI)  

def drawPoint(x,y):
    pythonGreen = "#476042"
    x1, y1 = (x - 5), (y - 5)
    x2, y2 = (x + 5), (y + 5)
    w.create_oval(x1, y1, x2, y2, width = 0, fill=pythonGreen)

def drawLine(p1,p2):
    w.create_line(p1[0], p1[1], p2[0], p2[1])

def drawRobot(x,y,heading): 
    scaled = []
    points = [(x+50,y+50), (x-50,y+50), (x-50,y-50), (x+50,y-50)]
    # print(points)
    for i in range(4):
        px = points[i][0] - x
        py = points[i][1] - y

        scaled.append((px * cos(heading) - py * sin(heading) + x, py * cos(heading) + px * sin(heading) + y ))
    w.create_oval(x-90,y-90,x+90,y+90)
    for i in range(3,-1,-1):
      drawLine(scaled[i],scaled[i-1])
      if i == 3:
        w.create_line(scaled[i][0], scaled[i][1], scaled[i-1][0], scaled[i-1][1], fill = "red")

#purePursuit/movement code

def moveToVel(target):
    global currRotation,x,y
    lkp = 0.04
    rkp = 0.08

    tx = target.getX
    ty = target.getY

    linearError = distToPoint(x,y,tx, ty)
    linearVel = linearError*lkp 

    currHeading = rtd(currRotation) if currRotation > 0 else rtd(currRotation) + 360 # 0-360
    targetHeading = 180-absoluteAngleToPoint(tx, ty) # -180-180

    # drawRobot(100,500,dtr(targetHeading))

    targetHeading = targetHeading if targetHeading >= 0 else abs(targetHeading) + 180

    dir = dirToSpin(targetHeading,currHeading)  
    
    # print(targetHeading,currHeading)
    # print(dir)

    rotationError = minError(targetHeading,currHeading)
    # print(diff)

    rotationVel = rotationError * rkp * dir 

    lVel = linearVel - rotationVel 
    rVel = linearVel + rotationVel 
    # print(lVel,rVel)
    return (lVel,rVel) 

def targetPoint(path, lookAhead, lineLookAhead, lineIndex):
    global x
    global y

    # farthestPoint = coordinate( path[lineLookAhead - 1].getX, path[lineLookAhead - 1].getY)
    targetPoint   = coordinate(0,0) 
    closestDist = 1000000000 

    a = lineIndex+lineLookAhead if lineLookAhead < (len(path) - lineIndex) else len(path) -1 
    for i in range(lineIndex, a):

        farthestPoint = coordinate( path[a-1].getX, path[a - 1].getY)
        drawLine((x,y),(farthestPoint.getX, farthestPoint.getY))
        # print(range(lineIndex, lineIndex+lineLookAhead if lineIndex+lineLookAhead < len(path) - lineIndex - 1 else len(path) - lineIndex - 1))

        x1 = path[i].getX
        y1 = path[i].getY
        x2 = path[i+1].getX
        y2 = path[i+1].getY

        ox1 = x1 - x 
        oy1 = y1 - y 
        ox2 = x2 - x 
        oy2 = y2 - y 

        dx = ox2-ox1 
        dy = oy2-oy1 
        dr = sqrt(pow(dx,2)+pow(dy,2)) 
        D = ox1*oy2 - ox2 * oy1 

        discriminant = pow(lookAhead,2)  *  pow(dr,2) - pow(D,2) 
        if (discriminant >= 0):
              sDiscriminant = sqrt(discriminant) 
              dxdy = D * dy 
              dxdx = D*dx 
              sdyxdxxsd = sign(dy) * dx * sDiscriminant 
              dr2 = pow(dr,2) 
              adyxsd = abs(dy) * sDiscriminant 

              minX = min(x1,x2) 
              maxX = max(x1,x2) 
              minY = min(y1,y2) 
              maxY = max(y1,y2) 

              sx1 = (dxdy + sdyxdxxsd) / dr2 
              sy1 = (-dxdx + adyxsd) / dr2 
              sx2 = (dxdy - sdyxdxxsd) / dr2 
              sy2 = (-dxdx - adyxsd) / dr2 

              s1= [sx1+x,sy1+y]
              s2 = [sx2+x,sy2+y]

              # drawPoint(s1[0],s1[1])
              # drawPoint(s2[0],s2[1])

              s1Valid = s1[0] >= minX and s1[0] <= maxX and s1[1] >= minY and s1[1] <= maxY 
              s2Valid = s2[0] >= minX and s2[0] <= maxX and s2[1] >= minY and s2[1] <= maxY 

              if i == a-1 and not (a-1 == len(path)):
                if(s1Valid):
                  return(1)
                if(s2Valid):
                  return(1)

              s1Dist = distToPoint(s1[0],s1[1],farthestPoint.getX,farthestPoint.getY) 
              s2Dist = distToPoint(s2[0],s2[1],farthestPoint.getX, farthestPoint.getY) 

              if (s1Valid and s1Dist < closestDist):
                targetPoint = coordinate(s1[0],s1[1]) 
                closestDist = s1Dist 

              if (s2Valid and s2Dist < closestDist):
                targetPoint  = coordinate(s2[0],s2[1]) 
                closestDist = s2Dist 
    
    return(targetPoint) 

def odomStep(dl,dr):
  global x,y,currRotation
  #currHeading -> -180-180
  offset = 25
  deltaRotation = (dl-dr) / 50
  if deltaRotation == 0:
    deltaY = dr
    deltaX = 0

  else:
    currRotation += deltaRotation
    r = dr/deltaRotation + offset

    relativeY = 2*sin(deltaRotation/2) * r 

    rotationOffset = currRotation+deltaRotation/2
    theta = PI/2
    radius = relativeY
    theta += rotationOffset
    deltaX = radius*cos(theta)
    deltaY = radius*sin(theta)
  x -= deltaX
  y -= deltaY

targetReached = False 

robotPath = []

def moveToPurePursuit(path, lookAhead,  lineLookAhead,finalTimeout):
    global lineIndex,targetReached,px,py,x,y,robotPath
    
    robotPath.extend( ((x,y),(px,py)))
    # robotPath.append((x,y))
    # robotPath.append((px,py))

    # if (pointInCircle(path[lineIndex + 1], lookAhead)):
    #     lineIndex += 1 

    target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)

    if target == 1:
      lineIndex += 1
      print(lineIndex)
      target = targetPoint(path,lookAhead, lineLookAhead, lineIndex)

    
    if lineIndex == len(path)-2:
      print("hi")
      targetReached = True

    if targetReached == True:
      target = path[-1]
      
    drawPoint(target.getX,target.getY)
    lVel,rVel = moveToVel(target) 
    print(lVel,rVel)
    odomStep(lVel,rVel)
    drawRobot(x,y,currRotation)

    px = x
    py = y
    
    

path = [(288, 792),(457, 599),(498, 380),(283, 330),(506, 188),(331, 136)]
p1 = [coordinate(288,792), coordinate(457,599), coordinate(498,380), coordinate(283,330), coordinate(506,188), coordinate(331,136)]

x,px = p1[0].getX, p1[0].getX 
y,py = p1[0].getY,p1[0].getY

#graphics 

points = []

canvas_width = 18*40 + 200
canvas_height = 18*40 + 100

master = Tk()
master.title("g")
w = Canvas(master, width=canvas_width, height=canvas_height)
w.create_rectangle(0 + 100 ,720 + 100 ,720+50 ,0+50) 
w.pack(expand=YES, fill=BOTH)

def step(event):
  global robotPath
  w.delete("all")
  w.create_rectangle(0 + 100 ,720 + 100 ,720+50 ,0+50) 
  for i in range(len(path)):
    drawPoint(path[i][0], path[i][1])
    try:
        drawLine((path[i][0], path[i][1]), (path[i+1][0], path[i+1][1]))
        
    except IndexError:
        pass
      
  for i in range(len(robotPath)):

    try:
        w.create_line((robotPath[i][0], robotPath[i][1]), (robotPath[i+1][0], robotPath[i+1][1]), fill = "blue") 
    except IndexError:
        pass

  moveToPurePursuit(p1,90,2,1000)

drawRobot(path[0][0], path[0][1],0)

for i in range(len(path)):
    drawPoint(path[i][0], path[i][1])
    try:
        drawLine((path[i][0], path[i][1]), (path[i+1][0], path[i+1][1]))
    except IndexError:
        pass

w.bind("<Button-1>", step)

mainloop()

