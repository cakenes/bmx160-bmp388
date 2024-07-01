#!/usr/bin/env python

from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import serial
import math
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=60)
ax = ay = az = 0.0

timestamp = 0
accel_x = accel_y = accel_z = 0.0
gyro_x = gyro_y = gyro_z = 0.0
mag_x = mag_y = mag_z = 0.0

timestamp = last_timestamp = rolling_avg_read_frequency = 0
last_pitch = last_roll = last_yaw = 0.0

GYRO_WEIGHT = 0.95
DELTA_WEIGHT = 0.05

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
    osd_text = "MagX: " + str(mag_x) + ", MagY: " + str(mag_y) + ", MagZ: " + str(mag_z)
    drawText((-3.5,-1.2, 2), osd_text)
    osd_text = "GyroX: " + str(gyro_x) + ", GyroY: " + str(gyro_y) + ", GyroZ: " + str(gyro_z)
    drawText((-3.5,-1.4, 2), osd_text)
    osd_text = "AccelX: " + str(accel_x) + ", AccelY: " + str(accel_y) + ", AccelZ: " + str(accel_z)
    drawText((-3.5,-1.6, 2), osd_text)
    osd_text = "C: Calibrate, R: Reset orientation, Esc: Quit."
    drawText((-3.5,-1.8, 2), osd_text)
    osd_text = "1: 100hz, 2: 80hz, 3: 50hz, 4: 10hz, Current Frequency: " + str(rolling_avg_read_frequency) + "hz"
    drawText((-3.5,-2, 2), osd_text)

    glRotatef(0, 0.0, 1.0, 0.0)
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
    global timestamp
    global accel_x, accel_y, accel_z
    global gyro_x, gyro_y, gyro_z
    global mag_x, mag_y, mag_z
    global last_timestamp, last_pitch, last_roll, last_yaw
    line = ser.readline()

    try:
        decoded_line = line.decode("utf-8", errors='replace').strip()
    except UnicodeDecodeError:
        print("Failed to decode line, skipping...")
        return
    
    angles = decoded_line.split(":")
    if len(angles) == 13:
        timestamp = float(angles[0])
        accel_x = float(angles[1])
        accel_y = float(angles[2])
        accel_z = float(angles[3])
        gyro_x = float(angles[4])
        gyro_y = float(angles[5])
        gyro_z = float(angles[6])
        mag_x = float(angles[7])
        mag_y = float(angles[8])
        mag_z = float(angles[9])
        time_elapsed = (timestamp - last_timestamp) / 1_000_000.0
        last_timestamp = timestamp


        pitch = math.atan2(-mag_x, mag_z)
        roll = math.atan2(mag_y, mag_z)
        pitch_degrees = math.degrees(pitch)
        roll_degrees = math.degrees(roll)

        delta_pitch = (pitch_degrees - last_pitch) * time_elapsed
        delta_roll = (roll_degrees - last_roll) * time_elapsed

        ay += (gyro_y * GYRO_WEIGHT - delta_roll * DELTA_WEIGHT)
        ax += (gyro_x * GYRO_WEIGHT + delta_pitch * DELTA_WEIGHT)
        az += (gyro_z)

        last_pitch = pitch_degrees
        last_roll = roll_degrees

        # Original calculation of pitch, roll, and yaw
        # pitch = math.atan2(float(angles[8]), math.sqrt(float(angles[7])**2 + float(angles[9])**2))
        # roll = math.atan2(float(angles[7]), math.sqrt(float(angles[8])**2 + float(angles[9])**2))
        # yaw = math.atan2(float(angles[2]), float(angles[1])) * (180 / math.pi)

        # pitch_degrees = math.degrees(pitch)
        # roll_degrees = math.degrees(roll)
        # yaw_degrees = math.degrees(yaw)

        # Calculate the change in angles
        # delta_pitch = (pitch_degrees - last_pitch) * time_elapsed
        # delta_roll = (roll_degrees - last_roll) * time_elapsed
        # delta_yaw = (yaw_degrees - last_yaw) * time_elapsed

        # Update ax, ay, az based on the change and time elapsed
        # ay += (gyro_y * GYRO_WEIGHT - delta_roll * DELTA_WEIGHT)
        # ax += (gyro_x * GYRO_WEIGHT + delta_pitch * DELTA_WEIGHT)
        # az += (gyro_z)

        # Update last_pitch, last_roll, last_yaw
        # last_pitch = pitch_degrees
        # last_roll = roll_degrees
        # last_yaw = yaw_degrees

def main():
    global ax, ay, az, rolling_avg_read_frequency
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
            
            rolling_avg_read_frequency = sum(read_counts) / len(read_counts)
            read_count = 0
            start_time = time.time()

        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()
            break 
        if event.type == KEYDOWN and event.key == K_r:
            ax = ay = az = 0.0
        elif event.type == KEYDOWN and event.key == K_c:
            ser.write(b'calibrate:100\n')
        elif event.type == KEYDOWN and event.key == K_1:
            ser.write(b'frequency:100\n')
        elif event.type == KEYDOWN and event.key == K_2:
            ser.write(b'frequency:80\n')
        elif event.type == KEYDOWN and event.key == K_3:
            ser.write(b'frequency:50\n')
        elif event.type == KEYDOWN and event.key == K_4:
            ser.write(b'frequency:10\n')
        read_data()
        draw()
        pygame.display.flip()

    ser.close()

if __name__ == '__main__': main()
