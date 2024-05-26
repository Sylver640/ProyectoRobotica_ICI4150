import time
import random
import serial
import json
import datetime

setPoint=100 #Set point for speed control, using ultrasonic sensor
time_PID = 0
integral = 0
time_prev = -1e-6
e_prev = 0
curSpeed = 0
deltat = 0.1

def PID(Kp, Ki, Kd, setpoint, measurement):
    global time_PID, integral, time_prev, e_prev
    # Value of offset - when the error is equal zero
    offset = 320

    # PID calculations
    e = setpoint - measurement

    P = Kp*e
    integral = integral + Ki*e*(time_PID - time_prev)
    D = Kd*(e - e_prev)/(time_PID - time_prev)
    # calculate manipulated variable - MV
    MV = offset + P + integral + D

    # update stored data for next iteration
    e_prev = e
    time_prev = time_PID
    
    return MV

def write_json_ultrasonido(data):
    x = {
        "sensor": "ultrasonido",
        "hora_captura": datetime.datetime.now(),
        "distancia": data
    }

    y = json.dumps(x)
    return

if __name__=="__main__": 
    n=100
    arduino = serial.Serial('COM5', 9600)
    for i in range(1, n):
        distance=float(arduino.readline())
        write_json_ultrasonido(distance)
        time_PID= i * deltat
        pid_out=PID(0.6,0.2,0.1,setPoint, distance)
        time_prev = time_PID

        print(pid_out)
    
        if pid_out>-1000:
            curSpeed=255 #velocidad alta
        
        if -2000<pid_out<-1000:
            curSpeed=100 #velocidad media
        
        if pid_out<-2000:
            curSpeed=50 #velocidad baja
            
        arduino.write(bytes([curSpeed]))
        
        time.sleep(0.5)
    arduino.close()