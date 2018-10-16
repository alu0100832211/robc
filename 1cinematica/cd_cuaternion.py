# -*- coding: utf-8 -*-
from math import *
# resolución de la cinemática directa mediante cuaterniones

# o1=Q1*(0,r1)*Q1c
# o2=Q1*Q2*(0,r2)*Q2c*Q1c + o1

# parametro primer eslabón
a1=10

# parametro segundo eslabón
a2=5


def multiplica_cuaternion(q1,q2):
        y = q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1]
        z = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2]
        w = q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3]
        x = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0]
        q=[x,y,z,w]
        return(q)


def suma_cuaternion(q1,q2):
        x = q1[0] + q2[0]
        y = q1[1] + q2[1]
        z = q1[2] + q2[2]
        w = q1[3] + q2[3]
        q=[x,y,z,w]
        return(q)


def cuaternion_rotacion(n,theta):
        # n es un vector
        # theta es un angulo
        x = cos(theta/2)
        y = n[0]*sin(theta/2)
        z = n[1]*sin(theta/2)
        w = n[2]*sin(theta/2)
        q=[x,y,z,w]
        return(q)


def cuaternion_conjugado(q1):
        x = q1[0]
        y = -q1[1]
        z = -q1[2]
        w = -q1[3]
        q=[x,y,z,w]
        return(q)


# cuaterniones de desplazamiento
r1=[0,a1,0,0]
r2=[0,a2,0,0]


# vectores de rotación
n1=[0,0,1]
n2=[0,0,1]


# introducción de las variables articulares
print('')
t1=input('valor de theta1 en grados  ')
t1=t1*pi/180
t2=input('valor de theta2 en grados  ')
t2=t2*pi/180


# calculo de los cuaterniones de rotación
q1=cuaternion_rotacion(n1,t1)
q1c=cuaternion_conjugado(q1)

q2=cuaternion_rotacion(n2,t2)
q2c=cuaternion_conjugado(q2)


# calculo del punto o1
i1=multiplica_cuaternion(q1,r1)
o1=multiplica_cuaternion(i1,q1c)
o1=[round(o1[0]),round(o1[1]),round(o1[2]),round(o1[3])]


# calculo del punto o2
i2=multiplica_cuaternion(q1,q2)
i2=multiplica_cuaternion(i2,r2)
i2=multiplica_cuaternion(i2,q2c)
i2=multiplica_cuaternion(i2,q1c)
o2=suma_cuaternion(o1,i2)
o2=[round(o2[0]),round(o2[1]),round(o2[2]),round(o2[3])]


# impresión de los resultados
print ''
print 'punto uno del robot'
print o1
print ''
print 'punto dos del robot'
print o2







