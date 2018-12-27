#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - Curso 2018/2019
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Filtros de particulas.

from math import *
from robot import *
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import select
from datetime import datetime
# ******************************************************************************
# Declaración de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def pinta(secuencia,args):
  # Dibujar una secuencia de puntos
  t = np.array(secuencia).T.tolist()
  plt.plot(t[0],t[1],args)

def mostrar(objetivos,trayectoria,trayectreal,filtro):
  # Mostrar mapa y trayectoria
  plt.ion() # modo interactivo
  plt.clf()
  plt.axis('equal')
  # Fijar los bordes del gráfico
  objT   = np.array(objetivos).T.tolist()
  bordes = [min(objT[0]),max(objT[0]),min(objT[1]),max(objT[1])]
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar mapa
  for p in filtro:
    dx = cos(p.orientation)*.05
    dy = sin(p.orientation)*.05
    plt.arrow(p.x,p.y,dx,dy,head_width=.05,head_length=.05,color='k')
  pinta(trayectoria,'--g')
  pinta(trayectreal,'-r')
  pinta(objetivos,'-.ob')
  p = hipotesis(filtro)
  dx = cos(p[2])*.05
  dy = sin(p[2])*.05
  plt.arrow(p[0],p[1],dx,dy,head_width=.075,head_length=.075,color='m')
  # Mostrar y comprobar pulsaciones de teclado:
  plt.draw()
#  if sys.stdin in select.select([sys.stdin],[],[],.01)[0]:
#    line = sys.stdin.readline()
  raw_input()

def genera_filtro(num_particulas, balizas, real, centro=[2,2], radio=3):
  # Inicialización de un filtro de tamaño 'num_particulas', cuyas partículas
  # imitan a la muestra dada y se distribuyen aleatoriamente sobre un área dada.
  pf = [robot() for x in range(num_particulas)]
  medidas = real.sense(balizas)
  for p in pf:
      x = random.uniform(centro[0]-radio, centro[0]+radio)
      y = random.uniform(centro[1]-radio, centro[1]+radio)
      p.set(x,y,real.sense(balizas)[-1])
      p.measurement_prob(medidas, balizas)
  return pf

def dispersion(filtro):
  # Dispersion espacial del filtro de particulas
    max_x = float("-inf")
    max_y = float("-inf")
    min_x = float("inf")
    min_y = float("inf")

    particula_min = []
    particula_max = []
    for particula in filtro:
        if (particula[0] > max_x and particula[1] > max_y):
            particula_max = particula
        elif (particula[0] > min_x and particula[1] > min_x):
            particula_min = particula

    return np.linalg.norm(np.subtract(particula_max, particula_min))

def peso_medio(filtro):
  # Peso medio normalizado del filtro de particulas
  mayor = max([p.weight for p in filtro])
  return sum([p.weight/mayor for p in filtro])/len(filtro)

# ******************************************************************************

random.seed(0)

# Definición del robot:
P_INICIAL = [0.,4.,0.] # Pose inicial (posición y orientacion)
V_LINEAL  = .7         # Velocidad lineal    (m/s)
V_ANGULAR = 140.       # Velocidad angular   (º/s)
FPS       = 10.        # Resolución temporal (fps)
HOLONOMICO = 0         # Robot holonómico
GIROPARADO = 0         # Si tiene que tener vel. lineal 0 para girar
LONGITUD   = .1        # Longitud del robot

N_PARTIC  = 50         # Tamaño del filtro de partículas
N_INICIAL = 2000       # Tamaño inicial del filtro

# Definición de trayectorias:
trayectorias = [
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.4*pi*i),2+2*cos(.4*pi*i)] for i in range(5)],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[2+2*sin(1.2*pi*i),2+2*cos(1.2*pi*i)] for i in range(5)],
    [[2*(i+1),4*(1+cos(pi*i))] for i in range(6)],
    [[2+.2*(22-i)*sin(.1*pi*i),2+.2*(22-i)*cos(.1*pi*i)] for i in range(20)],
    [[2+(22-i)/5*sin(.1*pi*i),2+(22-i)/5*cos(.1*pi*i)] for i in range(20)]
    ]

# Definición de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <índice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definición de constantes:
EPSILON = .1                # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma

real = robot()
real.set_noise(.01,.01,.01) # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

#inicialización del filtro de partículas y de la trayectoria
filtro = genera_filtro(N_INICIAL, objetivos, real)
filtro = resample(filtro, N_PARTIC)

trayectreal = [real.pose()]
trayectoria = [hipotesis(filtro)]

tiempo  = 0.
espacio = 0.
for punto in objetivos:
  while distancia(trayectoria[-1],punto) > EPSILON and len(trayectoria) <= 1000:

    #seleccionar pose
    pose = hipotesis(filtro)
    print "Mover a la derecha " + str(len(trayectreal)) + " pose: " + str(pose)

    w = angulo_rel(pose,punto)
    if w > W:  w =  W
    if w < -W: w = -W
    v = distancia(pose,punto)
    if (v > V): v = V
    if (v < 0): v = 0
    if HOLONOMICO:
      if GIROPARADO and abs(w) > .01:v = 0
      real.move(w,v)
    else:
      real.move_triciclo(w,v,LONGITUD)

    medidas = real.sense(objetivos)

    for p in filtro:
      p.move(w,v)
      p.measurement_prob(measurements=medidas, landmarks=objetivos)

    # Seleccionar hipótesis de localización y actualizar la trayectoria
    trayectreal.append(real.pose())
    trayectoria.append(hipotesis(filtro))
    mostrar(objetivos,trayectoria,trayectreal,filtro)

    # remuestreo
    #filtro = resample(filtro, N_INICIAL-len(trayectoria))

    espacio += v
    tiempo  += 1

if len(trayectoria) > 1000:
  print "<< ! >> Puede que no se haya alcanzado la posición final."
print "Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s"
print "Error medio de la trayectoria: "+str(round(sum(\
    [distancia(trayectoria[i],trayectreal[i])\
    for i in range(len(trayectoria))])/tiempo,3))+"m"
raw_input()

