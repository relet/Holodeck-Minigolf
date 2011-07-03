#!/usr/bin/env python
#affine tf, see: http://elonen.iki.fi/code/misc-notes/affine-fit/

import sys, pygame
import opencv
from opencv.highgui import *
import Image
import time, math, random
#from affinefit import *
from homography import *
import ode

## utils 

def sqr(a): 
  return a*a

def distance(p1,p2):
  return math.sqrt(sqr(p1[0]-p2[0])+sqr(p1[1]-p2[1]))

def angle(p1, p2):
  return math.atan2(p1[1]-p2[1], p1[0]-p2[0])

def center(pts): 
  cx = cy = 0.0
  for (x,y) in pts:
    cx+=x
    cy+=y
  cx /= len(pts)
  cy /= len(pts)
  return cx, cy

# init web camera
camera = cvCreateCameraCapture(0)
# init pygame drawing routines
pygame.init()
# init physics engine
odeWorld = ode.World()
odeWorld.setGravity((0,-9.8,0)) 
odeWorld.setERP(0.8)
odeWorld.setCFM(1E-5)

odeSpace = ode.Space()
odeFloor = ode.GeomPlane(odeSpace, (0,1,0), 0) 

odeObstacles=[]

def odeReset():
  #global odeSpace, odeFloor, odeObstacles
  #odeSpace = ode.Space()
  #odeFloor = ode.GeomPlane(odeSpace, (0,1,0), 0) 
  #odeObstacles = []
  pass

def setOdeWall(p1, p2):
  global odeSpace, odeObstacles
  dist = distance(p1,p2)
  a    = -angle(p1,p2)
  odeWall = ode.GeomBox(odeSpace, (dist, 10000, 0.01))
  cx,cy = center([p1,p2])
  odeWall.setPosition((cx, 0.0, cy))
  odeWall.setRotation((math.cos(a) , 0, math.sin(a),
                       0,            1, 0          ,
                       -math.sin(a), 0, math.cos(a)))
  return odeWall

def setOdeObstacle(poly):
  global odeObstacles
  for i in xrange(len(poly)-1):
    odeObstacles.append(setOdeWall(poly[i], poly[i+1]))
  if len(poly)>2: #do not close lines, but any poly >= 3
    odeObstacles.append(setOdeWall(poly[-1],poly[0]))
  pass

odeBBall = ode.Body(odeWorld)
M = ode.Mass()
M.setSphere(2500, 0.05)
odeBBall.setMass(M)

odeBall = ode.GeomSphere(odeSpace, radius=.05) #FIXME: confirm radius
odeBall.setBody(odeBBall)

contactgroup = ode.JointGroup()

fps = 50
dt = 1.0/fps

def near_callback(args, geom1, geom2):
  if not (type(geom1) == ode.GeomSphere or type(geom2) == ode.GeomSphere):
    return
  contacts = ode.collide(geom1, geom2)
  world, contactgroup = args
  for c in contacts:
    c.setBounce(1.00)
    c.setMu(0)
    j = ode.ContactJoint(world, contactgroup, c)
    j.attach(geom1.getBody(), geom2.getBody())

# some settings and constants
size = width, height = 640,480 
black = 0,    0,  0
red   = 55,  0,  0
white = 255,255,255
lgray = 230,230,230
gray  = 60 ,60 ,60
red   = 255,0  ,0  
green = 0  ,255,0 
dgreen= 0  ,80 ,0 
blue  = 0  ,0  ,255

action = ""

CCSIZE = 15 # size of calibration circles (dead border)
BALL   = 6
HOLE   = BALL+4
FRICTION = 0.99
#FRICTION = 0.999

ballx = bally = holex = holey = None
ballax = ballay = None
def resetGame():
  global ballx, bally, holex, holey, ballax, ballay 
  ballx = bally = holex = holey = ballax = ballay = None

def odeInit():
  global ballx, bally, ballax, ballay, odeBBall
  odeBBall.setPosition((ballx, 0.11, bally))
  odeBBall.setLinearVel((ballax, 0, ballay))
  odeBBall.setAngularVel((0, 0, 0))

screen = pygame.display.set_mode(size, pygame.DOUBLEBUF|pygame.FULLSCREEN)
#screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
pygame.display.set_caption("Virtual Minigolf")

MODE_INIT              = 0 # currently deprecated
MODE_DETECT_CIRCLES    = 1
MODE_DETECT_UNITSQUARE = 2
MODE_DETECT_OBSTACLES  = 3
MODE_DETECT_OBSTACLES2 = 4 
MODE_DETECT_OBSTACLES3 = 5 
MODE_READY             = 6 
MODE_SIMULATE          = 7

timer = time.time()

# all the useful methods 

def switchMode(m):
  global mode, timer
  mode = m
  timer = time.time()

switchMode(MODE_DETECT_CIRCLES)

def exit():
   sys.exit(0)
def calibrationCircle(x, y, r):
   #pygame.draw.circle(screen, black, (x,y), r+5)
   pygame.draw.circle(screen, white, (x,y), r)
   pygame.draw.circle(screen, black, (x,y), r-5)
def queryWebcam():
  return cvQueryFrame(camera)
def IPL2PIL(im):
  return opencv.adaptors.Ipl2PIL(im)
def displayImage(im):
  img = IPL2PIL(im)
  pgimg = pygame.image.frombuffer(img.tostring(), img.size, img.mode)
  screen.blit(pgimg, (0,0))

certainty = 0
corners = []
cameraSpace = []
gameSpace = []
detectionSpace = []

obstacles = []

isoRect = [(0,0),(1,0),(0,1),(1,1)]
isoRectIndex = 0
isoPointsClicked = 0



def extrude(pts, dist):
  cx, cy = center(pts)
  urn = []
  for (x,y) in pts:
    distc = distance((cx,cy),(x,y))
    x += (x-cx) * (dist/distc)
    y += (y-cy) * (dist/distc)
    urn.append((x,y))
  return urn

def calcCameraSpace(): 
  #compute four clusters
  dists = []
  allc = [c for quads in corners for c in quads]
  clusters = []
  for (px,py) in allc:
    assigned = False
    for i, (size,x,y) in enumerate(clusters):
      dist=math.sqrt(sqr(x-px)+sqr(y-py))
      if dist<10:
        assigned = True
        clusters[i] = (size+1.0,
                      (x*size+px)/(size+1.0), 
                      (y*size+py)/(size+1.0))
        break
    if not assigned:
      clusters.append((1.0,px,py))
  clusters.sort()
  return orderSquare([(c[1], c[2]) for c in clusters])

def setIsoRect(event):
  global isoRect, isoRectIndex, isoPointsClicked, timer
  isoRect[isoRectIndex] = event.dict["pos"]
  isoRectIndex = (isoRectIndex + 1)%4
  timer = time.time()
  isoPointsClicked += 1

def keyPress(event):
  global mode
  if event.dict["unicode"] == u'q':
    exit()
  if event.dict["unicode"] == u'r':
    resetGame()
    switchMode(MODE_DETECT_OBSTACLES)


def orderSquare(sq): 
  #sorts a quad of coordinates in clockwise order, starting at bottom left
  cx, cy =  center(sq)
  angles = [(math.atan2(x-cx, y-cy),x,y) for (x,y) in sq]
  angles.sort()
  return [(x,y) for (a,x,y) in angles]

projC2U = None #FIXME: deprecate the unused ones.
projU2C = None
projP2U = None
projU2P = None
projC2U = None
projC2P = None

def trans(poly, proj):
  #return [proj.Transform(p) for p in poly]
  return apply_homography(proj, poly)#.tolist()

def getProj(p1, p2):
  #return Affine_Fit(p1,p2)
  return find_homography(np.array(p1), np.array(p2))

def calculateTransformations():
  global isoRect, projC2U, projP2U, projU2P, projU2C, projC2U, projC2P
  global gameSpace, cameraSpace, detectionSpace

  unitRect =((0,0),(0,1),(1,1),(1,0))
  isoRect = orderSquare(isoRect)
  screenSize = ((0,0),(0,height),(width,height),(width,0))
  calSize = ((CCSIZE,CCSIZE),(CCSIZE,height-CCSIZE),(width-CCSIZE,height-CCSIZE),(width-CCSIZE,CCSIZE))

  projP2U = getProj(isoRect , unitRect) 
  projU2P = getProj(unitRect, isoRect )
  projP2C = getProj(calSize, cameraSpace)
  projC2P = getProj(cameraSpace, calSize)
  uRiCS   = trans(trans(unitRect, projU2P), projP2C)
  projC2U = getProj(uRiCS, unitRect)
  projU2C = getProj(unitRect, uRiCS)

  gameSpace = trans(cameraSpace, projC2U)

# determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs.
def point_inside_polygon(x,y,poly):

    n = len(poly)
    inside =False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

def retrieveObstacles(contours):
  global obstacles, odeObstacles
  obstc = []
  obstacles = []
  odeReset()

  for c in contours.hrange():
    count = c.total
    if (count < 3):
      continue
    ob = [(c[i].x,c[i].y) for i in xrange(count) if point_inside_polygon(c[i].x, c[i].y, cameraSpace)]
    if len(ob)>3:
      obstc.append(ob)
  for poly in obstc:
    oinu = trans(poly, projC2U)
    obstacles.append(oinu)
    setOdeObstacle(oinu)
  obstacles.append(gameSpace) #add the borders of the game!
  setOdeObstacle(gameSpace)

def getSafeLocation():
  for loop in xrange(100):
    bx = random.randint(CCSIZE+2*BALL,width-CCSIZE-2*BALL)
    by = random.randint(CCSIZE+2*BALL,height-CCSIZE-2*BALL)
    safe=True
    for ob in obstacles:
      for (x,y) in ob:
        px, py = u2p(x,y)
        if abs(px-bx)<2*BALL or abs(py-by)<2*BALL:
          safe=False
    if safe: break
  return bx,by


def initGame():
  global ballx, bally, holex, holey
  ball = getSafeLocation()
  for loop in xrange(1000):
    hole = getSafeLocation()
    if (ball[0]-hole[0] > 50) and (ball[1]-hole[1] > 50):
      break
  ballx, bally = ball
  holex, holey = hole
  ballx, bally   = trans([(ballx, bally)], projP2U)[0]
  holex, holey   = trans([(holex, holey)], projP2U)[0]

strokes = []
olddist = oldangle = 0 
lastx   = lasty    =  None
def registerStroke(pos):
  # TODO: make this take into account the frame rate / time elapsed!
  global strokes
  global olddist, oldangle, lastx, lasty
  px, py = pos
  bx, by = u2p(ballx, bally)
  dist =  distance(pos, (bx, by))
  #angl = angle(pos, (bx,by))  
  if dist < 2*BALL:
    strike()
    return 
  #if lastx == None:
  #  lastx, lasty = pos
  #move = distance(pos, (lastx, lasty))
  #if (move<200):
  if True:
    if dist>olddist*3/4: #unless we're moving quickly here
      strokes.append((bx-px, by-py))
      strokes  = strokes[-1:]
      olddist  = dist 
      #oldangle = angl
    lastx, lasty
    return True
  return False

swing = (0,0)
def calculateSwing():
  global swing
  if len(strokes)>0:
    swing = strokes[0]#center(strokes) #average actually

def strike():
  global ballx, bally, ballax, ballay, obstacles, holex, holey
  global action, swing, strokes
  calculateSwing() 
  ballax, ballay = swing[0]/0.5, swing[1]/0.5
  action = "STRIKE! %.2f %.2f" % swing
  #at this point, we move to unit space!
  zero           = trans([(0,0)], projP2U)[0]
  speed          = trans([(ballax, ballay)], projP2U)[0]
  ballax, ballay = speed[0]-zero[0], speed[1]-zero[1]
  odeInit()
  swing = (0,0)
  strokes = []
  switchMode(MODE_SIMULATE)

def u2p(px,py):
  if projU2P != None:
    return trans([(px,py)],projU2P)[0]
  else:
    return [px,py]


odeTimer = time.time()
def odeMove():
  global ballx, bally, ballax, bally
  global odeBall
  #sleep, if we happen to be faster than 50 fps (or whatever is defined)
  t = dt - (time.time()-odeTimer)
  if (t>0):
    time.sleep(t)
  
  #simulate
  n = 2
  for i in range(n):
    odeSpace.collide((odeWorld, contactgroup), near_callback)
    odeWorld.step(dt/n)
    contactgroup.empty()

  lasttime = time.time()
  ballx, none, bally = odeBall.getPosition()
  ballax, zvel, ballay = odeBBall.getLinearVel()
  #return to drawing routines
  ballax = ballax * FRICTION
  ballay = ballay * FRICTION
  odeBBall.setLinearVel((ballax, zvel, ballay))

  if distance((ballx, bally), (holex, holey)) < 0.25 and abs(ballax)<5 and abs(ballay)<5:
    resetGame()
    switchMode(MODE_DETECT_OBSTACLES)
    return 

  if abs(ballax)<0.01 and abs(ballay)<0.01:
    switchMode(MODE_READY)


im = queryWebcam()
igray = opencv.cvCreateImage(opencv.cvGetSize(im), 8, 1)
iwhite = opencv.cvCreateImage(opencv.cvGetSize(im), 8, 1)
stor = opencv.cvCreateMemStorage(0)

lastframe = time.time()
font = pygame.font.Font(None, 20)
while 1:
    for event in pygame.event.get():
      if event.type == pygame.QUIT: exit()
      if event.type == pygame.MOUSEBUTTONDOWN:
        if mode == MODE_DETECT_UNITSQUARE:
          setIsoRect(event)
        else:
          exit()
      if event.type == pygame.KEYDOWN: keyPress(event)
    #screen.fill(black)

    ### DO ANY VISION ##########################
    if mode == MODE_DETECT_OBSTACLES:
      screen.fill(white)
      queryWebcam()
      if time.time() > timer + 0.5:
        switchMode(MODE_DETECT_OBSTACLES2)
    elif mode == MODE_DETECT_OBSTACLES3:
      screen.fill(white)
      queryWebcam()
      if time.time() > timer + 0.5:
        action = ""
        switchMode(MODE_READY)
    elif mode == MODE_DETECT_OBSTACLES2:
      before = time.time()
      screen.fill(white)
      im = queryWebcam()
      opencv.cvCvtColor (im, igray, opencv.CV_BGR2GRAY)
      opencv.cvSmooth(igray, igray, opencv.CV_GAUSSIAN, 3, 3)
      opencv.cvAdaptiveThreshold(igray, iwhite, 255, opencv.CV_ADAPTIVE_THRESH_GAUSSIAN_C)
      num, contours = opencv.cvFindContours (iwhite, stor, opencv.sizeof_CvContour, opencv.CV_RETR_LIST)
  
      opencv.cvCvtColor(iwhite, im, opencv.CV_GRAY2BGR)
      staticImage = im

      retrieveObstacles(contours)
      switchMode(MODE_DETECT_OBSTACLES3)
      action = "Detection took: %.2f" % (time.time()-before)

    elif mode == MODE_READY:
      screen.fill(lgray)
      im  = queryWebcam()
      opencv.cvCvtColor(im, igray, opencv.CV_BGR2GRAY)
      opencv.cvSmooth(igray, igray, opencv.CV_GAUSSIAN, 5, 5)
      circles = opencv.cvHoughCircles(igray, stor, opencv.CV_HOUGH_GRADIENT, 2, height/4, 200, 150)

      #opencv.cvCvtColor(igray, im, opencv.CV_GRAY2BGR)
      #displayImage(im)

      for data in circles:
        circle = [(data[0]+3, data[1]+3)]
        if projC2U != None:
          cinu = trans(circle, projC2U)[0]
          cinp = trans(circle, projC2P)[0]
          pygame.draw.circle(screen, gray, cinp, 2)
          if distance(cinp, u2p(ballx, bally)) > 2 and distance(cinp, u2p(holex, holey)) > 2*BALL and point_inside_polygon(circle[0][0], circle[0][1], cameraSpace):
            if registerStroke(cinp):
              break
        else:
          pygame.draw.circle(screen, red, circle[0], 10)

    elif mode == MODE_DETECT_CIRCLES:
      screen.fill(black)
      im  = queryWebcam()
      opencv.cvCvtColor(im, igray, opencv.CV_BGR2GRAY)
      #detect circles
      opencv.cvSmooth(igray, igray, opencv.CV_GAUSSIAN, 5, 5)
      circles = opencv.cvHoughCircles(igray, stor, opencv.CV_HOUGH_GRADIENT, 2, height/4, 200, 100)

      #display blurred camera image for reference
      #opencv.cvSmooth(igray, igray, opencv.CV_GAUSSIAN, 35, 35)
      #opencv.cvCvtColor(igray, im, opencv.CV_GRAY2BGR)
      #displayImage(im)

      n = 0
      ptlist = []
      for data in circles:
        pygame.draw.circle(screen, white, (data[0]+3,data[1]+3), 2)
        ptlist.append((data[0]+3,data[1]+3))
        n+=1

      if n == 4:
        ptlist = orderSquare(ptlist)
        pygame.draw.polygon(screen, white, ptlist, 1)
        pygame.draw.circle(screen, white, center(ptlist), 1)
        certainty += 1
        corners.append(ptlist)

      if certainty == 30:
        cameraSpace = calcCameraSpace()
        if len(cameraSpace) == 4:
          switchMode(MODE_DETECT_UNITSQUARE)
        else:
          certainty=0
          corners=[]

    elif mode == MODE_DETECT_UNITSQUARE:
      if isoPointsClicked >= 4 and time.time() > timer + 5.0:
        calculateTransformations()
        switchMode(MODE_DETECT_OBSTACLES)

    ### DRAW SCREEN ############################
    if mode == MODE_SIMULATE: #we live in unit space here
      odeMove()
      if holex == None: #game has been reset
        continue
      screen.fill(lgray)
      for ob in obstacles:
        for p in xrange(1,len(ob)):
          pygame.draw.line(screen, red, u2p(ob[p][0], ob[p][1]), u2p(ob[p-1][0], ob[p-1][1]), 1)
        pygame.draw.line(screen, red, u2p(ob[0][0], ob[0][1]), u2p(ob[len(ob)-1][0], ob[len(ob)-1][1]), 1)
      pygame.draw.circle(screen, black, map(int,u2p(holex, holey)), HOLE)
      pygame.draw.circle(screen, white, map(int,u2p(ballx, bally)), BALL)
      pygame.draw.circle(screen, gray, map(int,u2p(ballx, bally)), BALL, 1)

    elif mode == MODE_READY:   
      if len(strokes)>0:
        stroke = center(strokes)
        bpos   = u2p(ballx, bally)
        pygame.draw.line(screen, red, bpos, (bpos[0]-stroke[0], bpos[1]-stroke[1]), 5)
      for ob in obstacles:
        for p in xrange(1,len(ob)):
          pygame.draw.line(screen, red, u2p(ob[p][0], ob[p][1]), u2p(ob[p-1][0], ob[p-1][1]), 1)
        pygame.draw.line(screen, red, u2p(ob[0][0], ob[0][1]), u2p(ob[len(ob)-1][0], ob[len(ob)-1][1]), 1)
      if obstacles:
        if ballx == None:
          initGame()
        pygame.draw.circle(screen, black, u2p(holex, holey), HOLE)
        pygame.draw.circle(screen, white, u2p(ballx, bally), BALL)
        pygame.draw.circle(screen, black, u2p(ballx, bally), BALL, 1)
    elif mode == MODE_INIT: 
      action = "Welcome to Virtual Minigolf!"
      displayImage(queryWebcam())
      if time.time() > timer + 5.0:
        timer = time.time()
        switchMode(MODE_DETECT_CIRCLES)
    elif mode == MODE_DETECT_UNITSQUARE:
      action = "Please highlight an even square on the surface."
      displayImage(queryWebcam())
      pygame.draw.polygon(screen, red, cameraSpace, 1)
      for i in xrange(4):
        if isoRect[i]:
          pygame.draw.circle(screen, blue, isoRect[i], 4)
    elif mode == MODE_DETECT_CIRCLES:
      action = "Please point the camera at the projection space."
      calibrationCircle(CCSIZE,CCSIZE,CCSIZE)
      calibrationCircle(width-CCSIZE,CCSIZE,CCSIZE)
      calibrationCircle(CCSIZE,height-CCSIZE,CCSIZE)
      calibrationCircle(width-CCSIZE,height-CCSIZE,CCSIZE)

    now = time.time()
    period = now - lastframe   
    fps = 1.000/period
    text = font.render("FPS: %.1f" % fps, 1, white)
    #text = font.render("dist: %.0f" % olddist, 1, white)
    #text = font.render("%s" % action, 1, white)
    textpos = text.get_rect(centerx=width/2)
    screen.blit(text, textpos)
    lastframe = now
    pygame.display.flip()
