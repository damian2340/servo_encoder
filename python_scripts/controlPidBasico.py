# -*- coding: utf-8 -*-
"""
Created on Wed May  5 01:56:43 2021

@author: damian

Comandos implementados en la placa:
V###.### ó v###.###:    establece la tensión de salida del controlador.
                        valor usado por el controlador para fijar la modulación por ancho de pulso: PWM = 0,5 (1+Vsalida/Vmaxima).
R####### ó R#######:    fija la resolución del encoder.
                        solo se usa para transformar los puloss a grados pero no esta implementado.
M###.### ó m###.###:    establece la máxima tesion soportada por el motor.
                        valor usado por el controlador para fijar la modulación por ancho de pulso: PWM = 0,5 (1+Vsalida/Vmaxima).
X ó x:                  reinicia la placa.

las ultimas seldas son la implementacion de:
    Linea 179 PID simple para controlar posición
    Linea 211 PID simple para controlar velocidad
    Linea 265 PID en cascada para controlar posición: El control mas rapido controla la velocidad del motor y
                                                      un pid mas lento controla la referencia de velocidad en funcion de la posición).
                                                              __________        __________    _________
                                                    REF -O---| PID pos |---O---| PID vel |---| Motor  |
                                                         |   |_________|   |   |_________|   |________|
                                                         |                 |______velocidad____|   |
                                                         |______________posicion___________________|
            
"""
#%% Librerias
from itertools import count
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import serial
import time
import threading

#%% Iniciar controlador serie
ser = serial.Serial(port='COM6', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=0.005, xonxoff=0, rtscts=0)
ser.close() 
ser.open()

#%% Funciones de interfaz con el motor y encoder
lastPos = 0
lastVel = 0 

def medir(puerto):
    global lastPos
    global lastVel
    try:                        
        s = puerto.readline(25)
        pos = float(s[0:9])
        vel = float(s[10:23])  
        lastPos = pos
        lastVel = vel
    except: #si no puede medir la velocidad devuelve el ultimo valor medido
        pos = lastPos
        vel = lastVel
    return pos,vel
    
def setvoltaje(voltaje, puerto):
    puerto.flushInput()
    send = bytes('V'+str(voltaje)[:10]+'\r'+'\n', 'utf-8')        
    puerto.write(send[0:10])
"""
 en el motorreductor la velocidad es relativamente proporcional a la tension exepto entre -2 y 2 volts que el motor no se mueve.
 para evitar que el controlador entregue esos valores se les puede sumar un offset a las tensiones menores a 2V
 (acelera la respuesta del controlador)"""
def setVoltajeCorregido(voltaje, puerto):
    if(voltaje > 0 ):
        voltaje += 2.6
        voltaje *= 12/14.6
    else:
        voltaje -= 2
        voltaje *= 12/14

    puerto.flushInput()
    send = bytes('V'+str(voltaje)[:10]+'\r'+'\n', 'utf-8')        
    puerto.write(send[0:10])
    
def resetControlador(puerto):
    global lastPos
    global lastVel 
    lastPos = 0
    lastVel = 0 
    puerto.write(bytes('X','utf-8')) 
    puerto.flushInput()

#%% Implemente una clase Pid basica igual a la que uso para control de motores
"""
  Pid paralelo con control de desborde implementado por multiplicacion de arrays
  Ver:https://arm-software.github.io/CMSIS_5/DSP/html/group__PID.html
"""
import numpy as np

class Pid():
    def __init__(self, kp = 0, ki = 0, kd =  0, maximo = 1, minimo = 0, setPoint = 0, deltaTiempo = 1, reset = False, nombre = ""):
        self.__nombre = nombre
        self.__setPoint = setPoint
        self.__maximo = maximo
        self.__minimo = minimo
        self.__error = np.zeros((1,3))  #historial de las 3 ultimas entradas de error
        self.__salida = 0.0
        if(deltaTiempo > 0):            
            self.setGanancias(kp , ki, kd, deltaTiempo)
        else:
            self.setGanancias(kp , ki, kd, 1)
            print("deltaTiempo debe ser mayor a \"0\"")
        if reset:
            self.reiniciar()
            
    def setGanancias(self, kp = 0, ki = 0, kd =  0, deltaTiempo = -1):
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        if(deltaTiempo > 0):
            self.__dt = deltaTiempo
        self.__ganancia = np.zeros((3,1))
        self.__ganancia[0,0] = self.__kp + self.__dt * self.__ki + self.__kd / self.__dt
        self.__ganancia[1,0] = -2 * self.__kd - self.__kp
        self.__ganancia[2,0] = self.__kd
        
    def reiniciar(self):
        self.__error[0,0] = 0
        self.__error[0,1] = 0
        self.__error[0,2] = 0
        self.__salida = 0
        
        
    def actualizarPid(self, nuevaEntrada):
        self.__error[0,2] = self.__error[0,1]
        self.__error[0,1] = self.__error[0,0]
        self.__error[0,0] = self.__setPoint - nuevaEntrada
        self.__salida = self.__salida + (self.__error.dot( self.__ganancia )[0,0])
        self.__acotarSalida() 
        return (self.__salida)

    def __acotarSalida(self):
        if ( self.__salida > self.__maximo ) :
            self.__salida = self.__maximo
        elif ( self.__salida  < self.__minimo ):
            self.__salida = self.__minimo
            
    @property
    def setPoint(self):
        return self.__setPoint

    @setPoint.setter
    def setPoint(self, setPoint):
        self.__setPoint = setPoint

    @property        
    def maximo(self):
        return self.__minimo

    @maximo.setter    
    def maximo(self, maximo):
        self.__maximo = maximo
        
    @property                
    def minimo(self):
        return self.__minimo
    
    @minimo.setter
    def minimo(self, minimo):
        self.__minimo = minimo
        
    @property
    def salida(self):
        return self.__salida
    
    @property
    def error0(self):
        return self.__error[0,0]
    
    @property
    def error1(self):
        return self.__error[0,1]
    
    @property
    def error2(self):
        return self.__error[0,2]
    
    
    @property
    def nombre(self):
        return self.__nombre


#%% CICLO DE CONTROL DE POSICION Simple
resetControlador(ser)

deltaTiempo = .01

controlPosicion = Pid(kp = 0.012, ki = 0.0, kd = 0.0000006, maximo = 12, minimo = -12, setPoint = 0, deltaTiempo = 0.01)
#las siguientes 2 lineas son redundantes aca. Solo estan para mostrar como cambiar los parametros mientras funciona el control
controlPosicion.setPoint = 200
controlPosicion.setGanancias(kp = 0.011, ki = 0.02, kd = 3e-6)

x_value = []
y_value = []
index = count()


def animatePos(i): 
    indice = next(index)
    posicion, velocidad = medir(ser)
    salidaControlador = controlPosicion.actualizarPid(posicion)
 #   setvoltaje(salidaControlador, ser)
    setVoltajeCorregido(salidaControlador, ser)
    x_value.append(indice*.01)
    y_value.append(posicion)
    plt.cla()
    plt.plot(x_value, y_value)


ani = FuncAnimation(plt.gcf(), animatePos, interval = deltaTiempo * 1000)
plt.tight_layout()
plt.show()
        
#%% CICLO DE CONTROL DE VELOCIDAD simple
resetControlador(ser)
time.sleep(2)
deltaTiempo = .005
setVelocidad = 2

controlVelocidad = Pid(kp = 0.48, ki = 2.4, kd = 0.00033, maximo = 12, minimo = -12, setPoint = setVelocidad, deltaTiempo = deltaTiempo, reset=(True))
#las siguientes 2 lineas son redundantes acá. Solo estan para mostrar como cambiar los parametros mientras funciona el control
controlVelocidad.setPoint = setVelocidad
controlVelocidad.setGanancias(kp = 0.48, ki = 2.4, kd = 0.00033)

tValue = []
velFiltred = []
velRef = []
index = count()

def animateVel(i):
    global tValue
    global velFiltred
    global velRef
    plt.cla()
    plt.plot(tValue, velFiltred, '.-', tValue, velRef)


def controlar(tValue, velFiltred, velRef, index, controlVelocidad):
    indice = 0
    inicio = time.time()
    while indice < 1200:
        ahora = time.time()
        if( (ahora - inicio ) > deltaTiempo):
            inicio += deltaTiempo
            indice = next(index)
            posicion, velocidad = medir(ser)
            ser.flushInput()
            
            tValue.append(indice*deltaTiempo)
            velFiltred.append(velocidad)
            velRef.append(controlVelocidad.setPoint)
            if(indice > 0):
                velFiltred[indice] = 0.4 * velocidad + 0.6 * velFiltred[indice-1]
            salidavoltaje = controlVelocidad.actualizarPid(velFiltred[indice])
            setVoltajeCorregido(salidavoltaje, ser)    

hiloControl = threading.Thread(name='controlar', 
                         target=controlar,
                         args=(tValue, velFiltred, velRef, index, controlVelocidad,),
                         daemon=True)

hiloControl.start()
ani = FuncAnimation(plt.gcf(), animateVel, interval = .2)
plt.tight_layout()
plt.show()

#%% Controlador de posición en cascada
""" 
Creo que para un sistema que responde tan lento no tiene mucho sentido pero se ve que llega mas rapido y con menos overshoot.
tambie se puede ver como el control de posicion hace que la velocidad siga a la referencia
Los sistemas de posisionamiento reales usan este sistema pero la posicion final nunca es un valor fijo, sino que la cambian con el tiempo para 
conseguir curvas de velocidades y aceleracione suaves.
"""
resetControlador(ser)
time.sleep(2)
deltaTiempo = .01
setVelocidad = 2

controlPosicion = Pid(kp = 0.0165, ki = 0.05, kd = 0.000003, maximo = 10, minimo = -10, setPoint = 500, deltaTiempo = deltaTiempo)
controlVelocidad = Pid(kp = 0.48, ki = 2.4, kd = 0.00033, maximo = 12, minimo = -12, setPoint = setVelocidad, deltaTiempo = deltaTiempo, reset=(True))
#las siguientes 2 lineas son redundantes acá. Solo estan para mostrar como cambiar los parametros mientras funciona el control
controlPosicion.setPoint = 500
controlVelocidad.setGanancias(kp = 0.48, ki = 2.4, kd = 0.00033)

tValue = []
posValue = []
velFiltred = []
velRef = []

fig, (ax1, ax2) = plt.subplots(2)
fig.suptitle('Posición y velocidad del motor')
def animateVel(i):
    global tValue
    global posValue
    global velFiltred
    global velRef
    plt.cla()
    global ax1
    global ax2
    ax1.plot(tValue, posValue)
    ax2.plot(tValue, velFiltred, '.', tValue, velRef, '.-')

index = count()
def controlar(tValue, posValue, velFiltred, velRef, index, controlVelocidad):
    indice = 0
    inicio = time.time()
    while indice < 1200:
        ahora = time.time()
        if( (ahora - inicio ) > deltaTiempo):
            inicio += deltaTiempo
            indice = next(index)
            posicion, velocidad = medir(ser)
            ser.flushInput()
            
            if(indice%5 == 0):
                controlVelocidad.setPoint = controlPosicion.actualizarPid(posicion)
                
            tValue.append(indice*deltaTiempo)
            posValue.append(posicion)
            velFiltred.append(velocidad)
            velRef.append(controlVelocidad.setPoint)
            """Como la velocidad tiene un poco de ruido hice un filtro pero no es realmente necesario"""
            if(indice > 0):
                velFiltred[indice] = 0.4 * velocidad + 0.6 * velFiltred[indice-1]
                
                
            salidavoltaje = controlVelocidad.actualizarPid(velFiltred[indice])
            setVoltajeCorregido(salidavoltaje, ser)    

hiloControl = threading.Thread(name='controlar', 
                         target=controlar,
                         args=(tValue, posValue, velFiltred, velRef, index, controlVelocidad,),
                         daemon=True)

hiloControl.start()
ani = FuncAnimation(plt.gcf(), animateVel, interval = .1)
plt.tight_layout()
plt.show()

#%% STRING DEVUELTO POR LA PLACA
#despues de mandar un comando para setear la tensión, la placa devuelve l aposicion y velocidad actuales.

ser.flushInput()
ser.write(bytes('V-2.25\n\r','utf-8'))
time.sleep(0.002)
s = ser.readline(25)
print (s )
        
#%% RESET
resetControlador(ser)

#%%
ser.close()             # close port