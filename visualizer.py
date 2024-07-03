#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import math
import time

ser = serial.Serial('/dev/ttyACM0', 921600, timeout=60)
ax = ay = az = 0.0

timestamp = 0
accelX = accelY = accelZ = 0.0
gyroX = gyroY = gyroZ = 0.0
magX = magY = magZ = 0.0

valueGyroX = valueGyroY = valueGyroZ = 0.0
valueMagX = valueMagY = valueMagZ = 0.0

timestamp = lastTimestamp = trueFrequency = 0

GYRO_WEIGHT = 0
MAG_WEIGHT = 1
ACCEL_WEIGHT = 0

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

    osd_text = "X: " + str(ax) + ", Y: " + str(ay) + ", Z: " + str(az)
    drawText((-3.5,-1.0, 2), osd_text)
    osd_text = "MagX: " + str(magX) + ", MagY: " + str(magY) + ", MagZ: " + str(magZ)
    drawText((-3.5,-1.2, 2), osd_text)
    osd_text = "GyroX: " + str(gyroX) + ", GyroY: " + str(gyroY) + ", GyroZ: " + str(gyroZ)
    drawText((-3.5,-1.4, 2), osd_text)
    osd_text = "AccelX: " + str(accelX) + ", AccelY: " + str(accelY) + ", AccelZ: " + str(accelZ)
    drawText((-3.5,-1.6, 2), osd_text)
    osd_text = "C: Calibrate, R: Reset orientation, Esc: Quit."
    drawText((-3.5,-1.8, 2), osd_text)
    osd_text = "1: 100hz, 2: 80hz, 3: 50hz, 4: 10hz, Actual Freq: " + str(trueFrequency) + "hz"
    drawText((-3.5,-2, 2), osd_text)

    glRotatef(az, 0.0, 1.0, 0.0)
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

def interpolate(value1, value2, t):
    return value1 + (value2 - value1) * (t * t * (3 - 2 * t))

def read_data():
    global ax, ay, az
    global timestamp, lastTimestamp
    global accelX, accelY, accelZ
    global gyroX, gyroY, gyroZ
    global magX, magY, magZ
    global valueGyroX, valueGyroY, valueGyroZ
    global valueMagX, valueMagY, valueMagZ
    global trueFrequency

    line = ser.readline()

    try:
        decoded_line = line.decode("utf-8", errors='replace').strip()
    except UnicodeDecodeError:
        print("Failed to decode line, skipping...")
        return
    
    angles = decoded_line.split(":")
    if len(angles) == 13:
        timestamp = float(angles[0])
        accelX = float(angles[1])
        accelY = float(angles[2])
        accelZ = float(angles[3])
        gyroX = float(angles[4])
        gyroY = float(angles[5])
        gyroZ = float(angles[6])
        magX = float(angles[7])
        magY = float(angles[8])
        magZ = float(angles[9])

        timeElapsed = (timestamp - lastTimestamp) / 1_000_000.0
        lastTimestamp = timestamp
        
        if MAG_WEIGHT != 0:
            angle_radians = math.atan2(-magY, magZ)
            angle_degrees_x = math.degrees(angle_radians)
            targetValueMagX = (450 - angle_degrees_x - 90) % 360
            valueMagX = interpolate(valueMagX, targetValueMagX, 0.2)

            angle_radians_y = math.atan2(-magX, magZ)
            angle_degrees_y = math.degrees(angle_radians_y)
            targetValueMagY = (450 - angle_degrees_y - 90) % 360
            valueMagY = interpolate(valueMagY, targetValueMagY, 0.2)

            if magZ > 0:
                valueMagZ = -180
            else:
                valueMagZ = 0

            # angle_radians = math.atan2(magY, magX)
            # angle_degrees_z = math.degrees(angle_radians)
            # targetValueMagZ = (450 - angle_degrees_z - 90) % 360
            # valueMagZ = interpolate(valueMagZ, targetValueMagZ, 0.2)

        
        if ACCEL_WEIGHT != 0:
            # Accelerometer wants to keep going to the direction its already going, only use mag data?. 
            accelPitch = math.atan2(accelY, math.sqrt(accelX**2 + accelZ**2))
            accelRoll = math.atan2(-accelX, math.sqrt(accelY**2 + accelZ**2))
            accelPitchDegrees = math.degrees(accelPitch)
            accelRollDegrees = math.degrees(accelRoll)
            # lastRoll, lastPitch missing, should be global
            deltaPitch = (accelPitchDegrees - lastPitch) * timeElapsed
            deltaRoll = (accelRollDegrees - lastRoll) * timeElapsed
            ay += deltaRoll
            ax += deltaPitch

        if GYRO_WEIGHT != 0:
            valueGyroX += gyroX * timeElapsed * trueFrequency / 10
            valueGyroY += gyroY * timeElapsed * trueFrequency / 10
            valueGyroZ += gyroZ * timeElapsed * trueFrequency / 10

        ax = (valueGyroX * GYRO_WEIGHT) + (valueMagX * MAG_WEIGHT)
        ay = (valueGyroY * GYRO_WEIGHT) + (valueMagY * MAG_WEIGHT)
        az = (valueGyroZ * GYRO_WEIGHT) + (valueMagZ * MAG_WEIGHT)

def main():
    global trueFrequency
    global valueMagX, valueMagY, valueMagZ
    video_flags = OPENGL|DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((920,480), video_flags)
    resize(920,480)
    init()
    read_count = 0
    start_time = time.time()
    read_counts = []

    while 1:
        read_count += 1
        if time.time() - start_time >= 1:
            read_counts.append(read_count)
            if len(read_counts) > 10:
                read_counts.pop(0)
            
            trueFrequency = sum(read_counts) / len(read_counts)
            read_count = 0
            start_time = time.time()

        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break 
        if event.type == KEYDOWN and event.key == K_r:
            valueMagX = valueMagY = valueMagZ = 0.0
        elif event.type == KEYDOWN and event.key == K_c:
            ser.write(b'calibrate 100\n')
        elif event.type == KEYDOWN and event.key == K_1:
            trueFrequency = 100
            ser.write(b'frequency 100\n')
        elif event.type == KEYDOWN and event.key == K_2:
            trueFrequency = 80
            ser.write(b'frequency 80\n')
        elif event.type == KEYDOWN and event.key == K_3:
            trueFrequency = 50
            ser.write(b'frequency 50\n')
        elif event.type == KEYDOWN and event.key == K_4:
            trueFrequency = 10
            ser.write(b'frequency 10\n')
        read_data()
        draw()
        pygame.display.flip()

    ser.close()

if __name__ == '__main__': main()
