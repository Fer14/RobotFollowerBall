#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
# import numpy as np
import sim

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    # Camera handles
    _,cam = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_camera',
                                        sim.simx_opmode_oneshot_wait)
    sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    sim.simxReadVisionSensor(clientID, cam, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, cam

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------

# def getImage(clientID, hRobot):
#     img = []
#     err,r,i = sim.simxGetVisionSensorImage(clientID, hRobot[2], 0,
#                                             sim.simx_opmode_buffer)

#     if err == sim.simx_return_ok:
#         img = np.array(i, dtype=np.uint8)
#         img.resize([r[1],r[0],3])
#         img = np.flipud(img)
#         img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

#     return err, img

# --------------------------------------------------------------------------

def getImageBlob(clientID, hRobot):
    rc,ds,pk = sim.simxReadVisionSensor(clientID, hRobot[2],
                                         sim.simx_opmode_buffer)
    blobs = 0
    coord = []
    if rc == sim.simx_return_ok and pk[1][0]:
        blobs = int(pk[1][0])
        offset = int(pk[1][1])
        for i in range(blobs):
            coord.append(pk[1][4+offset*i])
            coord.append(pk[1][5+offset*i])

    return blobs, coord


#------------------------------------------

def avoid(sonar, lastSeenBall):
    if (sonar[2] < 0.15) or (sonar[3] < 0.15) or (sonar[1] < 0.15) or (sonar[0] < 0.15):
        print('OBJETO A IZQ CERCA')
        return +0.9, -0.9, True
    
    elif (sonar[4] < 0.15) or (sonar[5] < 0.15) or (sonar[6] < 0.15) or (sonar[7] <0.15): #or (sonar[2] < 0.3):
        print('OBJETO A DCHA CERCA')
        return -0.9, +0.9, True
    
    elif (sonar[2] < 0.2) or (sonar[3] < 0.2) or (sonar[1] < 0.2) or (sonar[0] < 0.2):
        if lastSeenBall < 0.5:
            print('OBJETO A IZQ')
            return +1.5, +1.5, True
    
    elif (sonar[4] < 0.2) or (sonar[5] < 0.2) or (sonar[6] < 0.2) or (sonar[7] <0.2): #or (sonar[2] < 0.3):
        if lastSeenBall > 0.5:
            print('OBJETO A DCHA')
            return +1.5, +1.5, True
    
    return +1.0, +1.0, False

#------------------------------------------------------------------

def followBall(coord, lspeed, rspeed, distanceBall):


    # Cuando está muy cerca, decelero
    if coord[1]>0.7:
        return +0.2, +0.2

    else: #Si no, centro la bola
        if coord[0] < 0.4:
            rspeed += 0.6
        elif coord[0] > 0.6:
            lspeed += 0.6
        else:
            lspeed += 2.0
            rspeed += 2.0

    return lspeed, rspeed

#------------------------------------------------------------------

def searchBall(lspeed,rspeed,lastSeenBall):
    if lastSeenBall < 0.5:
        lspeed += -0.5
        rspeed += 0.9
    else:
        lspeed += 0.9
        rspeed += -0.5
    return lspeed, rspeed



# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))


    # Horizontal position of the ball
    lastSeenBall = 0
    # Record of last 5 vertical position (distance) of the ball
    distanceBall = [0, 0, 0, 0, 0]
    # true if there is something close to avoid
    somethingClose = False

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID) ##hRobot contains '[lmh, rmh], sonar, cam'


        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            # print '### s', sonar

            blobs, coord = getImageBlob(clientID, hRobot)

            distanceBall.pop(0)
            if blobs != 0 and len(coord) != 0:
                lastSeenBall = coord[0]
                distanceBall.append(coord[1])
                
            else:
                distanceBall.append(0)
        
            
            if lastSeenBall:
                # Se comprueba si hay que esquivar algo
                lspeed, rspeed, somethingClose = avoid(sonar, lastSeenBall)
                # Si no hay que esquivar nada, o bien se sigue la bola o bien se busca
                if not somethingClose: 
                    if blobs:
                        lspeed, rspeed = followBall(coord, lspeed, rspeed, distanceBall) 
                    else:
                        lspeed, rspeed = searchBall(lspeed, rspeed, lastSeenBall)
                    
            else:
                # Estado inicial para buscar la bola
                lspeed, rspeed = -2.0, +2.0


            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.01)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
