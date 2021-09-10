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

# --------------------------------------------------------------------------
'''def avoid(sonar):
    if (sonar[3] < 0.5) or (sonar[4] < 0.5):
        lspeed, rspeed = +0.3, -0.5
    elif sonar[1] < 0.5:
        lspeed, rspeed = +1.0, +0.3
    elif sonar[5] < 0.5:
        lspeed, rspeed = +0.2, +0.7
    else:
        lspeed, rspeed = +2.0, +2.0

    return lspeed, rspeed'''

#-----------------------------------------------------------------------------


def turn(direction):

    lspeed,rspeed = 0,0

    if direction == 'left' :
        lspeed,rspeed = +0.5,0.5
    else:
        lspeed,rspeed = 0.5,+0.5

    return lspeed,rspeed

#-------------------------------------------------------------------------------

'''def avoid(sonar, ball, coord):

    print('Sonar IZQ----------------------------------')
    print('Sonar 2',sonar[1])
    print('Sonar 3',sonar[2])
    print('Sonar 4',sonar[3])
    print('Sonar DCHA----------------------------------')
    print('Sonar 5',sonar[4])
    print('Sonar 6',sonar[5])
    print('Sonar 7',sonar[6])
    print('#############################################')


    if ((sonar[2] < 0.35) or (sonar[3] < 0.35) or (sonar[1] < 0.35)) and ((sonar[4] < 0.35) or (sonar[5] < 0.35) or (sonar[6] < 0.35)): #or (sonar[2] < 0.3):
        if  (sonar[2] < sonar[5]) or (sonar[1] < sonar[6]):
            lspeed, rspeed = turn('left')
            #print('Girar IZQ 1')
        else:
            lspeed, rspeed = turn('right')
            #print('Girar DCHA 1')

    elif (sonar[4] < 0.35) or (sonar[5] < 0.35) or (sonar[6] < 0.35):# or (sonar[7] < 0.4): GIRAR DERECHA SI DETECTA OBJ EN IZQUIERDA
        lspeed, rspeed = turn('right')
        #print('Girar DCHA 2')

    elif (sonar[2] < 0.35) or (sonar[3] < 0.35) or (sonar[1] < 0.35): #or (sonar[2] < 0.3): GIRAR IZQUIERDA SI DETECTA OBJ EN DCHA
        lspeed, rspeed = turn('left')
        #print('Girar IZQ 2') 

    else:
        lspeed, rspeed = +2.0, +2.0

    return lspeed, rspeed

'''


#------------------------------------------

def avoid(sonar,lspeed,rspeed):

    if (sonar[2] < 0.3) or (sonar[3] < 0.3) or (sonar[1] < 0.3) or (sonar[0] < 0.2) :
        lspeed = +0.5
        rspeed = -0.5
        print('OBJETO A IZQ')
    
    elif (sonar[4] < 0.3) or (sonar[5] < 0.3) or (sonar[6] < 0.3) or (sonar[7] <0.2): #or (sonar[2] < 0.3):
        lspeed = -0.5
        rspeed = +0.5
        print('OBJETO A DCHA')


    return lspeed,rspeed

#------------------------------------------------------------------

def followBall(ball,coord,lspeed,rspeed,lastSeenBall):

    if ball: #Si veo la pelota, seguirla 
        if coord[1]>0.7: #Cuando está muy cerca, paro
            lspeed = 0.2;
            rspeed = 0.2;

        else: #Si no está tan cerca
            if coord[0] < 0.3:
                lspeed -= 0.4
                rspeed += 0.4
            elif coord[0] > 0.7:
                lspeed += 0.4
                rspeed -= 0.4
            else:
                lspeed, rspeed = +1, +1
        

    else:#Si he dejado de ver la pelota
        if lastSeenBall>0: #La ha visto antes ya
            if lastSeenBall < 0.1:
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


    lastSeenBall = 0

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
        
            #BY DEFECT
            lspeed, rspeed = +1, +1


            #IF I SEE A BALL
            if blobs != 0 and len(coord)!=0:
                print('BALL IN (y): ', coord[1])
                #print('#####################')
                lastSeenBall = coord[0]
                #print('LAST SEEN BALL: ',lastSeenBall)
            
            lspeed,rspeed = followBall(blobs,coord,lspeed,rspeed, lastSeenBall)

            lspeed, rspeed = avoid(sonar,lspeed,rspeed)


            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
