# -*- coding: utf-8 -*-
import sys
from math import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Funciones de mostrar en cinemática directa
def ramal(I,prev=[],base=0):
  # Convierte el robot a una secuencia de puntos para representar
  O = []
  if I:
    if isinstance(I[0][0],list):
      for j in range(len(I[0])):
        O.extend(ramal(I[0][j], prev, base or j < len(I[0])-1))
    else:
      O = [I[0]]
      O.extend(ramal(I[1:],I[0],base))
      if base:
        O.append(prev)
  return O

def muestra_robot(O,ef=[]):
  # Pinta en 3D
  OR = ramal(O)
  OT = np.array(OR).T
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  # Bounding box cúbico para simular el ratio de aspecto correcto
  max_range = np.array([OT[0].max()-OT[0].min()
                       ,OT[1].max()-OT[1].min()
                       ,OT[2].max()-OT[2].min()
                       ]).max()
  Xb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten()
     + 0.5*(OT[0].max()+OT[0].min()))
  Yb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten()
     + 0.5*(OT[1].max()+OT[1].min()))
  Zb = (0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten()
     + 0.5*(OT[2].max()+OT[2].min()))
  for xb, yb, zb in zip(Xb, Yb, Zb):
     ax.plot([xb], [yb], [zb], 'w')
  ax.plot3D(OT[0],OT[1],OT[2],marker='s')
  ax.plot3D([0],[0],[0],marker='o',color='k',ms=10)
  if not ef:
    ef = OR[-1]
  ax.plot3D([ef[0]],[ef[1]],[ef[2]],marker='s',color='r')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.show()
  return


  #######################################################################
  ######   FIN DE FUNCIONES DE CINEMÁTICA DIRECTA 	#####################
  #######################################################################

# resoluci�n de la cinem�tica directa mediante cuaterniones

# o1=Q1*(0,r1)*Q1c
# o2=Q1*Q2*(0,r2)*Q2c*Q1c + o1

# parametro primer eslab�n
a1 = 8

# parametro segundo eslab�n
a2 = 5

# Parámetro tercer eslabón
a3 = 5


def multiplica_cuaternion(q1,q2):
	y = q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1]
	z = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2]
	w = q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3]
	x = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0]
	q = [x, y, z, w]
	return (q)


def suma_cuaternion(q1,q2):
	x = q1[0] + q2[0]
	y = q1[1] + q2[1]
	z = q1[2] + q2[2]
	w = q1[3] + q2[3]
	q = [x, y, z, w]
	return (q)


def cuaternion_rotacion(n,theta):
	x = cos(theta/2)
	y = n[0]*sin(theta/2)
	z = n[1]*sin(theta/2)
	w = n[2]*sin(theta/2)
	q = [x, y, z, w]
	return (q)


def cuaternion_conjugado(q1):
	x = q1[0]
	y = -q1[1]
	z = -q1[2]
	w = -q1[3]
	q = [x, y, z, w]
	return (q)


def cuaternion_unitario(q):
	cuaterion_abs = sqrt(q[0]** 2 + q[1]** 2 + q[2]** 2 + q[3]** 2)
	x = q[0] / cuaterion_abs
	y = q[1] / cuaterion_abs
	z = q[2] / cuaterion_abs
	w = q[3] / cuaterion_abs
	q = [x, y, z, w]
	return (q)


# Cuaterniones de desplazamiento
r1 = [0, a1, 0, 0]
r2 = [0, a2, 0, 0]
r3 = [0, a3, 0, 0]


# Vectores de rotación
n1 = [0, 0, 1]
n2 = [0, 0, 1]
n3 = [0, 0, 1]


# Introducción de las variables articulares
t1=float(input('\nValor de theta1 en grados: '))
t1=t1*pi/180
t2=float(input('Valor de theta2 en grados: '))
t2 = t2 * pi / 180
t3=float(input('Valor de theta3 en grados: '))
t3=t3*pi/180


# Cálculo de los cuaterniones de rotación
q1=cuaternion_rotacion(n1,t1)
q1c=cuaternion_conjugado(q1)

q2=cuaternion_rotacion(n2,t2)
q2c = cuaternion_conjugado(q2)

q3 = cuaternion_rotacion(n3, t3)
q3c = cuaternion_conjugado(q3)


# Cálculo del punto o1
i1=multiplica_cuaternion(q1,r1)
o1=multiplica_cuaternion(i1,q1c)
o1=[round(o1[0]),round(o1[1]),round(o1[2]),round(o1[3])]


# Cálculo del punto o2
i2=multiplica_cuaternion(q1,q2)
i2=multiplica_cuaternion(i2,r2)
i2=multiplica_cuaternion(i2,q2c)
i2=multiplica_cuaternion(i2,q1c)
o2=suma_cuaternion(o1,i2)
o2=[round(o2[0]),round(o2[1]),round(o2[2]),round(o2[3])]


# Cálculo del punto o3
i3 = multiplica_cuaternion(q1,q2)
i3 = multiplica_cuaternion(i3,q3)
i3 = multiplica_cuaternion(i3,r3)
i3 = multiplica_cuaternion(i3,q3c)
i3 = multiplica_cuaternion(i3,q2c)
i3 = multiplica_cuaternion(i3, q1c)
o3 = suma_cuaternion(i3, o2)
o3 = [round(o3[0]),round(o3[1]),round(o3[2]),round(o3[3])]

# Impresión de los resultados
print (f'\nPunto O1: {o1}')
print (f'Punto O2: {o2}')
print (f'Punto O3: {o3}')

o0 = [0, 0, 0, 1]
o1 = [o1[1], o1[2], o1[3], 1]
o2 = [o2[1], o2[2], o2[3], 1]
o3 = [o3[1], o3[2], o3[3], 1]

muestra_robot([o0, o1, o2, o3])
