import vpython as vp
from vpython import canvas,vector, color, box, sphere, rate, norm, cylinder
import random
import math
import time
import numpy as np


def main():
    side = 50.0
    thk = 0.3

    rad = 5
    leng = rad*5
    ax = vector(0,1,0)
    s3 = 2 * side + thk

    start = vector(-side, 1, -side)
    end = vector(side, 1, side)
    zero = vector(0,20, 0)

    #transformation matric between frames 0 and 1.
    #hardcoded beacuse it is the only transformation with an x translation
    P0_P1 = np.array([[1, 0, 0],
                      [0, 1, 0.3],
                      [0, 0, 1]])

    scene2 = canvas(title='Collision Avoidance',
                    width=1800,
                    height=900,
                    center=zero,
                    background=color.gray(.2),
                    forward=vector(0.5,-2,0),
                    fov=70)

    floor = box(pos=vector(0, 0, 0),
               size=vector(s3, thk, s3),
               color=color.gray(1))

    base = cylinder(axis=vector(0,1,0),
                    pos=vector(0, thk, 0),
                    color=color.red,
                    radius=3,
                    length=5)

    joint1 = sphere(pos=vector(0, base.pos.y  + base.length + 1,0), radius=2.9, color=color.white)
    arm1 = cylinder(axis=vector(0,1,0), pos=vector(0,joint1.pos.y, 0), color=color.red,
                    length=10 + joint1.radius)

    joint2 = sphere(pos=vector(0,arm1.pos.y + arm1.length,0),
                    radius=1.5,
                    color=color.white)

    arm2 = cylinder(axis=vector(0,1,0),
                    pos=vector(0,joint2.pos.y,0),
                    color=color.red,
                    length=5 + joint2.radius)

    joint3 = sphere(pos=vector(0,arm2.pos.y + arm2.length,0),
                    radius=1.5,
                    color=color.white)
    
    arm3 = cylinder(axis=vector(0,1,0),
                    pos=vector(0,joint3.pos.y,0),
                    color=color.red,
                    length=5 + joint3.radius)

    effector = sphere(pos=vector(0,arm3.pos.y + arm3.length,0),
                      radius=0.5,
                      color=color.white,
                      make_trail=True,
                      retain=400)

    degree = 0
    reachedEnd = False

    Ts = []
    P1_P2 = createAdjacentTx_Ty(vp.radians(45), base.length + 1)
    P2_P3 = createAdjacentTx_Ty(vp.radians(45), arm1.length)
    P3_P4 = createAdjacentTx_Ty(vp.radians(45), arm2.length)
    P4_P5 = createAdjacentTx_Ty(vp.radians(0), arm3.length)



    P0_5 = end_effector(P0_P1, P1_P2, P2_P3, P3_P4, P4_P5)

    
    # print(P0_5)
    time.sleep(5)
    for theta in range(0,1):
        rate(10)


        joint2.pos = vector(0,P0_5[2][1][2],P0_5[2][0][2])


        joint3.pos = vector(0,P0_5[3][1][2],P0_5[3][0][2])


        effector.pos = vector(0,P0_5[4][1][2],P0_5[4][0][2])









        
    # print(effector.pos)



#pass in theta between two frames in degrees, and length of arm between the frames
def createAdjacentTx_Ty(theta, length):
    #used roatation matrix for clockwise rotations
    Tx_Ty = np.array([[np.cos(theta), np.sin(theta), 0],
                      [-np.sin(theta), np.cos(theta), length],
                      [0, 0, 1]])
    return Tx_Ty

def end_effector(P0_P1, P1_P2, P2_P3, P3_P4, P4_P5):
    T0_2 = np.matmul(P0_P1, P1_P2)
    T0_3 = np.matmul(T0_2, P2_P3)
    T0_4 = np.matmul(T0_3, P3_P4)
    T0_5 = np.matmul(T0_4,  P4_P5)
    # T0_5 = P0_P1.dot(P1_P2).dot(P2_P3).dot(P3_P4).dot(P4_P5)

    return [P0_P1, T0_2, T0_3,T0_4, T0_5]

if __name__ == '__main__':
    main()


