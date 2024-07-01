#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import math

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
ax = ay = az = 0.0
last_read = 0.0

def resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawText(position, textString):     
    font = pygame.font.SysFont ("Courier", 18, True)
    textSurface = font.render(textString, True, (255,255,255,255), (0,0,0,255))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos3d(*position)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
    
    glLoadIdentity()
    glTranslatef(0,0.0,-7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax) + " Press R to reset orientation")
    drawText((-2,-2, 2), osd_text)

    glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay ,1.0,0.0,0.0)
    glRotatef(-1*ax ,0.0,0.0,1.0)

    glBegin(GL_QUADS)	
    glColor3f(0.0,1.0,0.0)
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f( 1.0, 0.2, 1.0)		

    glColor3f(1.0,0.5,0.0)	
    glVertex3f( 1.0,-0.2, 1.0)
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		

    glColor3f(1.0,0.0,0.0)		
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2, 1.0)		

    glColor3f(1.0,1.0,0.0)	
    glVertex3f( 1.0,-0.2,-1.0)
    glVertex3f(-1.0,-0.2,-1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f( 1.0, 0.2,-1.0)		

    glColor3f(0.0,0.0,1.0)	
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2,-1.0)		
    glVertex3f(-1.0,-0.2,-1.0)		
    glVertex3f(-1.0,-0.2, 1.0)		

    glColor3f(1.0,0.0,1.0)	
    glVertex3f( 1.0, 0.2,-1.0)
    glVertex3f( 1.0, 0.2, 1.0)
    glVertex3f( 1.0,-0.2, 1.0)		
    glVertex3f( 1.0,-0.2,-1.0)		
    glEnd()	
         
def read_data():
    global ax, ay, az
    global last_read
    line = ser.readline()
    if line:
        decoded_line = line.decode("utf-8")
        angles = decoded_line.split(":")
        if len(angles) == 13:
            if last_read != float(angles[0]):
                last_read = float(angles[0])
                # pitch = math.atan2(float(angles[7]), math.sqrt(float(angles[8]) ** 2 + float(angles[9]) ** 2))
                # roll = math.atan2(float(angles[8]), math.sqrt(float(angles[7]) ** 2 + float(angles[9]) ** 2))
                # yaw = math.atan2(math.sqrt(float(angles[7]) ** 2 + float(angles[8]) ** 2), float(angles[9]))
                # pitch_degrees = math.degrees(pitch)
                # roll_degrees = math.degrees(roll)
                # yaw_degrees = math.degrees(yaw)
                # ax += (float(angles[4]) / 10 * 0.96) + (pitch_degrees / 10 * 0.04)
                # ay += (float(angles[5]) / 10 * 0.96) + (roll_degrees / 10 * 0.04)
                # az += (float(angles[6]) / 10 * 0.96) + (yaw_degrees / 10 * 0.04)
                ax += (float(angles[4]) / 12)
                ay += (float(angles[5]) / 12)
                az += (float(angles[6]) / 12)

def main():
    global ax, ay, az
    video_flags = OPENGL|DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((920,480), video_flags)
    resize(920,480)
    init()
    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break 
        if event.type == KEYDOWN and event.key == K_r:
            ax = ay = az = 0.0
        read_data()
        draw()
        pygame.display.flip()
    ser.close()

if __name__ == '__main__': main()
