#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
from bluetooth import *
#import socket
#import binascii
#from array import array
from struct import *
import collections
import itertools
import numpy as np
import math
import transformations as tf
#import pyautogui, sys
from pymouse import PyMouse

class RingBuffer (object):
    """Ring buffer"""
    def __init__ (self, size = 4096):
            self._buf = collections.deque (maxlen = size)

    def put (self, data):
            """Adds data to the end of the buffer"""
            self._buf.extend (data)

    def get (self, size):
            """Retrieves data from the beginning of the buffer"""
            data = str ()
            for i in xrange (size):
                    data += self._buf.popleft ()
            return data

    def peek (self, size):
            """\"Peeks\" at the beginning of the buffer (i.e.: retrieves data without removing them from the buffer)"""
            return str (bytearray (itertools.islice (self._buf, size)))

    def len (self):
            """Returns the length of the buffer"""
            return len (self._buf)

BUP = False
BDOWN = False
BLEFT = False
timestamp = 0
gyrox = gyroy = gyroz = 0
accelx = accely = accelz = 0
lasttimestamp = 0
lastgyrox = lastgyroy = lastgyroz = 0
lastaccelx = lastaccely = lastaccelz = 0
dx = 0.0
dy = 0.0
dz = 0.0
vx = 0.0
vy = 0.0
vz = 0.0
vix = 0.0
viy = 0.0
viz = 0.0
xpos = 0.0
ypos = 0.0
zpos = 0.0
rotx = 0.0
roty = 0.0
rotz = 0.0
gx = 0.0
gy = 0.0
gz = 0.0
aRealx = 0.0
aRealy = 0.0
aRealz = 0.0
aWorldx = 0.0
aWorldy = 0.0
aWorldz = 0.0
movfactor = 2.0
frames = 0
locked = False
poslimit = 10

# Create the client socket
client_socket=BluetoothSocket(RFCOMM) #BLUETOOTH
BoxMat  =  (GLfloat * 16)()
OffMat  =  (GLfloat * 16)()
ActualPosMat  =  (GLfloat * 16)()
BoxQuat =  (GLfloat *  4)()
CameraAxis=(GLfloat *  9)()
DataFifo = RingBuffer()

#obj = OBJ("/home/medios/test.obj", swapyz=False)

def toMatrixQ(q):
    w,x,y,z = q[0], q[1], q[2], q[3]
    xx = 2.0*x*x
    yy = 2.0*y*y
    zz = 2.0*z*z
    xy = 2.0*x*y
    zw = 2.0*z*w
    xz = 2.0*x*z
    yw = 2.0*y*w
    yz = 2.0*y*z
    xw = 2.0*x*w
    return ( 1.0-yy-zz, xy-zw,     xz+yw,     0.0,
             xy+zw,     1.0-xx-zz, yz-xw,     0.0,
             xz-yw,     yz+xw,     1.0-xx-yy, 0.0,
             0.0,       0.0,       0.0,       1.0)

def resize((width, height)):
    global BoxMat,OffMat

    if height==0:
        height=1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glGetFloatv(GL_MODELVIEW_MATRIX, BoxMat) 
    glGetFloatv(GL_MODELVIEW_MATRIX, OffMat) 

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def render_grid():
    global CameraAxis
    glPushMatrix()
    glLoadIdentity()
    gluLookAt(*CameraAxis)
    glLineWidth(1)
    glColor3f(0.5, 0.5, 0.5)
    glBegin(GL_LINES)
    for i in range(-10, 11):
        glVertex3f(i, -10.0, 0.0)
        glVertex3f(i, 10.0, 0.0)
    for i in range(-10, 11):
        glVertex3f(-10.0, i, 0.0)
        glVertex3f(10.0, i, 0.0)
    glEnd()
    glPopMatrix()

def drawText(position, textString):     
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos3d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def drawaxis():
    global CameraAxis
    glPushMatrix()
    glLoadIdentity()
    gluLookAt(*CameraAxis)
    glBegin(GL_LINES);
    #draw line for x axis
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(10.0, 0.0, 0.0)
    #draw line for y axis
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 10.0, 0.0)
    #draw line for Z axis
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 10.0)
    glEnd()
    glPopMatrix()

    #abc = square size, m = rot. matrix, xyz = origin, oa,ob,oc = offsets
    #lock = rotate over itself or over world, RGB = color ig C==1
def draw_square(a,b,c,m,x,y,z,oa,ob,oc,lock,R,G,B,C):
    global CameraAxis,OffMat,ActualPosMat
    glPushMatrix()
    glLoadIdentity()
    gluLookAt(*CameraAxis)
    if lock: glTranslatef(x,y,z)    
    glMultMatrixf(OffMat)
    glMultMatrixf(m)
    if not lock: glTranslatef(x,y,z)
    glGetFloatv(GL_MODELVIEW_MATRIX, ActualPosMat)
    glBegin(GL_QUADS)	
    if C: glColor3f(R,G,B)
    else: glColor3f(0.0,1.0,0.0)
    glVertex3f( a+oa, b+ob,-c+oc)
    glVertex3f(-a+oa, b+ob,-c+oc)		
    glVertex3f(-a+oa, b+ob, c+oc)		
    glVertex3f( a+oa, b+ob, c+oc)		

    if C: glColor3f(R,G,B)
    else: glColor3f(1.0,0.5,0.0)	
    glVertex3f( a+oa,-b+ob, c+oc)
    glVertex3f(-a+oa,-b+ob, c+oc)		
    glVertex3f(-a+oa,-b+ob,-c+oc)		
    glVertex3f( a+oa,-b+ob,-c+oc)		

    if C: glColor3f(R,G,B)
    else: glColor3f(1.0,0.0,0.0)		
    glVertex3f( a+oa, b+ob, c+oc)
    glVertex3f(-a+oa, b+ob, c+oc)		
    glVertex3f(-a+oa,-b+ob, c+oc)		
    glVertex3f( a+oa,-b+ob, c+oc)		

    if C: glColor3f(R,G,B)
    else: glColor3f(1.0,1.0,0.0)	
    glVertex3f( a+oa,-b+ob,-c+oc)
    glVertex3f(-a+oa,-b+ob,-c+oc)
    glVertex3f(-a+oa, b+ob,-c+oc)		
    glVertex3f( a+oa, b+ob,-c+oc)		

    if C: glColor3f(R,G,B)
    else: glColor3f(0.0,0.0,1.0)	
    glVertex3f(-a+oa, b+ob, c+oc)
    glVertex3f(-a+oa, b+ob,-c+oc)		
    glVertex3f(-a+oa,-b+ob,-c+oc)		
    glVertex3f(-a+oa,-b+ob, c+oc)		

    if C: glColor3f(R,G,B)
    else: glColor3f(1.0,0.0,1.0)	
    glVertex3f( a+oa, b+ob,-c+oc)
    glVertex3f( a+oa, b+ob, c+oc)
    glVertex3f( a+oa,-b+ob, c+oc)		
    glVertex3f( a+oa,-b+ob,-c+oc)		
    glEnd()
    glPopMatrix()
  
def draw():
    global rquad
    global BoxMat,OffMat,BoxQuat
    global CameraAxis, timestamp,movfactor
    global gyrox,gyroy,gyroz,accelx,accely,accelz,timestamp,lasttimestamp
    global lastgyrox,lastgyroy,lastgyroz,lastaccelx,lastaccely,lastaccelz,rotx,roty,rotz
    global dx,dy,dz,vx,vy,vz,vix,viy,viz,xpos,ypos,zpos,gx,gy,gz,aRealx,aRealy,aRealz,aWorldx,aWorldy,aWorldz
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
    glPushMatrix()
    glLoadIdentity()
    glMultMatrixf(toMatrixQ(BoxQuat))
    glGetFloatv(GL_MODELVIEW_MATRIX, BoxMat) 
    glPopMatrix()
    render_grid()
    drawaxis()
    #draw_square(0.4,1.0,0.2,BoxMat,1,1,2,0,0,0,1)
    if BUP: buph = 0.15
    else: buph = 0.25
    if BLEFT: blefth = 0.15
    else: blefth = 0.25
    draw_square(1.0,0.4,0.2,BoxMat,xpos*movfactor,ypos*movfactor,zpos*movfactor+2,0  ,0   ,0     ,True,0,0,0,0)
    #draw_square(100,0.1,0.1,BoxMat,xpos*movfactor,ypos*movfactor,zpos*movfactor+2,0  ,0   ,0     ,True,0.2,0.2,0.2,1)
    draw_square(0.1,0.1,0.1,BoxMat,xpos*movfactor,ypos*movfactor,zpos*movfactor+2,0.8,-0.2,buph  ,True,0,0,1,1)
    draw_square(0.1,0.1,0.1,BoxMat,xpos*movfactor,ypos*movfactor,zpos*movfactor+2,0.7,0.1 ,blefth,True,1,1,1,1)
    osd_line1  = "Timestamp:  " + str("{}".format(timestamp))
    osd_line2  = "dT (ms):    " + str("{}".format(timestamp-lasttimestamp))
    osd_line3  = "X raw accel:" + str("{}".format(accelx))
    osd_line4  = "Y raw accel:" + str("{}".format(accely))
    osd_line5  = "Z raw accel:" + str("{}".format(accelz))
    osd_line6  = "X pos:      " + str("{}".format(xpos))
    osd_line7  = "Y pos:      " + str("{}".format(ypos))
    osd_line8  = "Z pos:      " + str("{}".format(zpos))
    osd_line9  = "Gravity X:  " + str("{}".format(gx))
    osd_line10 = "Gravity Y:  " + str("{}".format(gy))
    osd_line11 = "Gravity Z:  " + str("{}".format(gz))
    osd_line12 = "Roll:       " + str("{}".format(rotx))
    osd_line13 = "Pitch:      " + str("{}".format(roty))
    osd_line14 = "Yaw:        " + str("{}".format(rotz))
    osd_line15 = "X accel ng: " + str("{}".format(aRealx))
    osd_line16 = "Y accel ng: " + str("{}".format(aRealy))
    osd_line17 = "Z accel ng: " + str("{}".format(aRealz))
    osd_line18 = "Speed factor:  " + str("{}".format(movfactor))
    #drawText((-3.7,1.95,-5), osd_line1)
    drawText((-1.9,-1.7,-5), osd_line2)
    """drawText((-3.7,1.75,-5), osd_line3)
    drawText((-3.7,1.65,-5), osd_line4)
    drawText((-3.7,1.55,-5), osd_line5)
    drawText((-3.7,1.45,-5), osd_line6)------
    drawText((-3.7,1.35,-5), osd_line7)
    drawText((-3.7,1.25,-5), osd_line8)
    drawText((-3.7,1.15,-5), osd_line9)
    drawText((-3.7,1.05,-5), osd_line10)
    drawText((-3.7,0.95,-5), osd_line11)
    drawText((-3.7,0.84,-5), osd_line12)
    drawText((-3.7,0.74,-5), osd_line13)
    drawText((-3.7,0.64,-5), osd_line14)
    drawText((-3.7,0.54,-5), osd_line15)
    drawText((-3.7,0.44,-5), osd_line16)
    drawText((-3.7,0.34,-5), osd_line17)"""
    drawText((-1,-1.7,-5), osd_line18)

def breadln(self):
    message = ""
    while True:
        chunk = self.recv(1)
        if chunk == "" or chunk == "\n":
            break
        message += chunk
    return message

def readfifo(self):
    global DataFifo
    fifo = bytearray()
    leave = False
    while not leave:
        chunk = self.recv(4096)
        DataFifo.put(chunk)
        while DataFifo.len()>49: 
            fifo = DataFifo.get(49)
            #print binascii.hexlify(fifo)
            if ((ord(fifo[46])&0xF8)==0) and (fifo[47]=="A") and (fifo[48]=="A"):
            #if (fifo[47]=="A") and (fifo[48]=="A"):
                leave = True
            else:
                trash = DataFifo.get(1)
                #print "trash"
    return fifo
  
def parsefifo(fifo): #BoxQuat order w y x z
    global BoxQuat,OffMat,ActualPosMat,BUP,BDOWN,BLEFT,timestamp,lasttimestamp,locked,frames
    global gyrox,gyroy,gyroz,accelx,accely,accelz,gx,gy,gz,aRealx,aRealy,aRealz,aWorldx,aWorldy,aWorldz
    global lastgyrox,lastgyroy,lastgyroz,lastaccelx,lastaccely,lastaccelz
    global dx,dy,dz,vx,vy,vz,vix,viy,viz,xpos,ypos,zpos,poslimit,rotx,roty,rotz
    Mat1 = (GLfloat * 16)()
    Mat2 = (GLfloat * 16)()
    lasttimestamp = timestamp
    lastaccelx = accelx
    lastaccely = accely
    lastaccelz = accelz
    lastgyrox = gyrox
    lastgyroy = gyroy
    lastgyroz = gyroz
    data0,data1,data2,data3,gyrox,gyroy,gyroz,accelx,accely,accelz,s,timestamp,buttons,s2 = struct.unpack('>iiiiiiiiii2sIc2s',fifo)
    accelx = ((accelx / 65535.0)/8192)*9.81
    accely = ((accely / 65535.0)/8192)*9.81
    accelz = ((accelz / 65535.0)/8192)*9.81
    gyrox = gyrox / 65535.0
    gyroy = gyroy / 65535.0
    gyroz = gyroz / 65535.0
    BoxQuat[0] = data0 / 1073741824.0 #w
    BoxQuat[1] = data2 / 1073741824.0 #y
    BoxQuat[2] = data1 / 1073741824.0 #x
    BoxQuat[3] = data3 / 1073741824.0 #z
    BUP = (ord(buttons)>>2)&1 == 0
    BDOWN = (ord(buttons)>>1)&1 == 0
    BLEFT = ord(buttons)&1 == 0
    #print binascii.hexlify(fifo)
    #print "QUAT: ",BoxQuat[0] ,BoxQuat[2] ,BoxQuat[1] ,BoxQuat[3]
    gx = 2 * (BoxQuat[2]*BoxQuat[3] - BoxQuat[0]*BoxQuat[1])*9.81
    gy = 2 * (BoxQuat[0]*BoxQuat[2] + BoxQuat[1]*BoxQuat[3])*9.81
    gz = (BoxQuat[0]*BoxQuat[0] - BoxQuat[2]*BoxQuat[2] - BoxQuat[1]*BoxQuat[1] + BoxQuat[3]*BoxQuat[3])*9.81
    aRealx = (accelx - gx)
    aRealy = (accely - gy)
    aRealz = (accelz - gz)
    
    rot = np.array([[ActualPosMat[0],ActualPosMat[1],ActualPosMat[2],ActualPosMat[3]], \
          [ActualPosMat[4],ActualPosMat[5],ActualPosMat[6],ActualPosMat[7]],[ActualPosMat[8], \
          ActualPosMat[9],ActualPosMat[10],ActualPosMat[11]],[ActualPosMat[12], \
          ActualPosMat[13],ActualPosMat[14],ActualPosMat[15]]])
    rot = tf.euler_from_matrix(rot,'sxyz')
    rotx = rot[0] -1
    roty = rot[1]
    rotz = rot[2]+math.pi/2
    """print "rotx: ",rotx*180.0 / math.pi
    print "roty: ",roty*180.0 / math.pi
    print "rotz: ",rotz*180.0 / math.pi
    print " " """

    if True:
        dt = (timestamp-lasttimestamp)/1000.0
        vix = vx
        viy = vy
        viz = vz
        vx = vix + (accely*math.cos(rotz) + accelx*math.sin(rotz))*dt
        vy = viy + (accely*(-math.sin(rotz)) + accelx*math.cos(rotz))*dt
        vz = viz + accelz*dt
        dx = (((vix + vx)/2.0)*dt)
        dy = (((viy + vy)/2.0)*dt)
        dz = (((viz + vz)/2.0)*dt)
        if locked or frames<2:
            xpos = 0
            ypos = 0
            zpos = 0
            vx = 0
            vy = 0
            vz = 0
        else:
            xpos = xpos + dx
            ypos = ypos + dy
            zpos = 0 #zpos + dz
        limit = poslimit/movfactor
        if xpos>limit:
            xpos = limit
            vx = 0
        elif xpos<-limit:
            xpos = -limit
            vx = 0
        if ypos>limit:
            ypos = limit
            vy = 0
        elif ypos<-limit:
            ypos = -limit
            vy = 0
        if zpos>limit:
            zpos = limit
            vz = 0
        elif zpos<-limit:
            zpos = -limit
            vz = 0

def read_data():
    fifo = bytearray(42)
    fifo = readfifo(client_socket)
    parsefifo(fifo)

def transpose(m):
    mt = [m[0],m[4],m[8],m[12],m[1],m[5],m[9],m[13],m[2],m[6],m[10],m[14],m[3],m[7],m[11],m[15]]
    return mt

def main():
    global CameraAxis,BoxQuat,OffMat,blu,frames,locked,movfactor
    global dx,dy,dz,vx,vy,vz,vix,viy,viz,xpos,ypos,zpos
    client_socket.connect(("98:D3:31:80:74:F8", 1))
    video_flags = OPENGL|DOUBLEBUF
    pygame.init()
    dispinfo = pygame.display.Info()
    #print (dispinfo)
    scw = 1280#dispinfo.current_w
    sch = 700#dispinfo.current_h
    screen = pygame.display.set_mode((scw,sch), video_flags)
    pygame.display.set_caption("Press Esc to quit")
    pygame.key.set_repeat(100,100)
    resize((scw,sch))
    #CameraAxis =  [-10,1,5,0,0,2,0,0,1]
    CameraAxis =  [-20,0,15,0,0,2,0,0,1]
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    once = 0
    leftclicked = False
    rightclicked = False
    isdown = False
    offx = 0
    offy = 0
    mxoff = 0
    myoff = 0
    m = PyMouse()

    while 1:
        if (once == 0) and (frames == 1):
            once =1
            OffMat = transpose(toMatrixQ(BoxQuat))
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break      
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_UP):
            CameraAxis[0] = CameraAxis[0] + 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_DOWN):
            CameraAxis[0] = CameraAxis[0] - 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_LEFT):
            CameraAxis[1] = CameraAxis[1] + 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_RIGHT):
            CameraAxis[1] = CameraAxis[1] - 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_q):
            CameraAxis[2] = CameraAxis[2] + 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_a):
            CameraAxis[2] = CameraAxis[2] - 0.5
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_PLUS):
            movfactor = movfactor + 0.25
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_MINUS):
            movfactor = movfactor - 0.25
        if (event.type == QUIT or (event.type == KEYDOWN and event.key == K_SPACE)) or BUP:
            OffMat = transpose(toMatrixQ(BoxQuat))
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_o):
            OffMat = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_l):
            if locked:
                locked = False
            else:
                locked = True
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_b):
            dx = 0
            dy = 0
            dz = 0
            vx = 0
            vy = 0
            vz = 0
            vix = 0
            viy = 0
            viz = 0
            xpos = 0
            ypos = 0
            zpos = 0
        read_data()
        draw()

        #print mx,"  ",my
        if True: #BDOWN:
            mx, my = m.position() #gets mouse current position coordinates
            #newx = mx-accelx*3
            #newy = my-accely*3
            newx = dispinfo.current_w*1.5*math.sin(rotz) + dispinfo.current_w/2
            newy = dispinfo.current_h*2*math.sin(rotx) + dispinfo.current_h/2
            m.move(newx,newy)

        if BLEFT: 
            newx,newy = m.position()
            m.click(newx,newy,1)
            #pyautogui.mouseDown(button='left')
            #leftclicked = True
            #print "triggered..."
        #if (not BLEFT) and leftclicked:
            #pyautogui.mouseUp(button='left')
            #leftclicked = False
            #print "triggered..."
        #if BUP: 
            #m.click(newx,newy,2)
            #pyautogui.mouseDown(button='right')
            #rightclicked = True
            #print "triggered..."
        #if (not BUP) and rightclicked: 
            #pyautogui.mouseUp(button='right')
            #rightclicked = False
            #print "triggered..."

        pygame.display.flip()
        frames = frames+1

    print "fps:  %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks))

if __name__ == '__main__': main()
