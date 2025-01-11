from vpython import *
from time import *
import numpy as np
import math 
import serial
ad=serial.Serial('COM4',9600)
sleep(1)

scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(-1,-1,-1)
scene.width=600
scene.height=600
xArrow=arrow(length=2.5,shaftwidth=.1,color=color.red,axis=vector(1,0,0))
yArrow=arrow(length=2.5,shaftwidth=.1,color=color.green,axis=vector(0,1,0))
zArrow=arrow(length=2.5,shaftwidth=.1,color=color.blue,axis=vector(0,0,1))

xDyArrow=arrow(length=2,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
yDyArrow=arrow(length=2,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
zDyArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))
while True:
    rate(20) 
    while ad.inWaiting()==0:
        pass
    dataPacket=ad.readline()
    dataPacket=str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(",")
    Roll=float(splitPacket[0])*toRad
    Pitch=float(splitPacket[1])*toRad
    Yaw=float(splitPacket[2])*toRad

    
    i=vector((cos(Yaw)*cos(Pitch))*toDeg,(cos(Pitch)*sin(Yaw))*toDeg,(-sin(Pitch))*toDeg)
    j=vector((sin(Pitch)*sin(Roll)*cos(Yaw)-cos(Roll)*sin(Yaw))*toDeg,(sin(Roll)*sin(Pitch)*sin(Yaw)+cos(Roll)*cos(Yaw))*toDeg,(sin(Roll)*cos(Pitch))*toDeg)
    k=vector((cos(Roll)*sin(Pitch)*cos(Yaw)+sin(Roll)*sin(Yaw))*toDeg,(cos(Roll)*sin(Pitch)*sin(Yaw)-sin(Roll)*cos(Yaw))*toDeg,(cos(Roll)*cos(Pitch))*toDeg)

    xDyArrow.axis=i
    yDyArrow.axis=j
    zDyArrow.axis=k