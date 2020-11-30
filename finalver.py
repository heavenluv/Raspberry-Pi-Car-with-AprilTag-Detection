
import time
import threading
import RPi.GPIO as GPIO
import numpy
import cv2
import apriltag


EA, I2, I1, EB, I4, I3, LS, RS = (13, 19, 26, 16, 20, 21, 6, 12)
FREQUENCY = 50
GPIO.setmode(GPIO.BCM)
GPIO.setup([EA, I2, I1, EB, I4, I3], GPIO.OUT)
GPIO.setup([LS, RS],GPIO.IN)
GPIO.output([EA, I2, EB, I3], GPIO.LOW)
GPIO.output([I1, I4], GPIO.HIGH)

pwma = GPIO.PWM(EA, FREQUENCY)
pwmb = GPIO.PWM(EB, FREQUENCY)
pwma.start(0)
pwmb.start(0)
lspeed = 0
rspeed = 0
lcounter = 0
rcounter = 0


class PID:

    def __init__(self, P=1.8, I=0.01, D=2.4, speed=2, duty=10):

        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.err_pre = 0
        self.err_last = 0
        self.u = 0
        self.integral = 0
        self.ideal_speed = speed
        self.last_duty = duty
        self.pre_duty = duty
	
    def update(self,feedback_value):
        self.err_pre = self.ideal_speed - feedback_value
        self.integral+= self.err_pre
        self.u = self.Kp*self.err_pre + self.Ki*self.integral + self.Kd*(self.err_pre-self.err_last)
        self.err_last = self.err_pre
        self.pre_duty = self.last_duty + self.u
        if self.pre_duty > 100:
            self.pre_duty = 100
        elif self.pre_duty < 0:
            self.pre_duty = 0
            self.last_duty = self.pre_duty
        return self.pre_duty

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain
		

def my_callback(channel):
    global lcounter
    global rcounter
    if (channel==LS):
        lcounter+=1
    elif(channel==RS):
        rcounter+=1

            
def getspeed():
    global rspeed
    global lspeed
    global lcounter
    global rcounter
    GPIO.add_event_detect(LS,GPIO.RISING,callback=my_callback)
    GPIO.add_event_detect(RS,GPIO.RISING,callback=my_callback)
    while True:
        rspeed=(rcounter/10.00)
        lspeed=(lcounter/10.00)
        rcounter = 0
        lcounter = 0
        time.sleep(1)
   

   
thread2=threading.Thread(target=getspeed)
thread2.start()


def scanner():
    global tagid
    cap = cv2.VideoCapture(0)  # 获取摄像头句柄,只连接一个摄像头时参数写0即可
    alw=1
    tagid=2
    while True:
        ret, frame = cap.read()  # 读一帧
        cv2.imshow("display", frame)  # 显示
        if alw == 1:
            cv2.imwrite("image.jpg", frame)    
            img = cv2.imread('image.jpg',cv2.IMREAD_GRAYSCALE)
            detector = apriltag.Detector()
            result= detector.detect(img)
            if len(result)!= 0:
                if result[0][1] == tagid and tagid == 1:
                	L_control.ideal_speed=0
                    R_control.ideal_speed=0
                	time.sleep(2)
                    L_control.ideal_speed=2
                    R_control.ideal_speed=2
                	continue
                if result[0][1] == tagid and tagid == 0:
                	L_control.ideal_speed=0
                    R_control.ideal_speed=0
                	time.sleep(2)
                    L_control.ideal_speed=2
                    R_control.ideal_speed=2
                	continue
                print(result[0][1])
                tagid=result[0][1]
                if tagid == 0 :           	
                    time.sleep(0.5)
                    L_control.ideal_speed=0
                    R_control.ideal_speed=1
                    time.sleep(2)
                    L_control.ideal_speed=0
                    R_control.ideal_speed=0
                    time.sleep(2)
                    L_control.ideal_speed=2
                    R_control.ideal_speed=2
                    print("right")
                    	pass
                elif tagid == 1:
                    time.sleep(0.5)
                    L_control.ideal_speed=1
                    R_control.ideal_speed=0
                    time.sleep(2)
                    L_control.ideal_speed=0
                    R_control.ideal_speed=0
                    time.sleep(2)
                    L_control.ideal_speed=2
                    R_control.ideal_speed=2
                    print("left")
                elif tagid == 2:
                    L_control.ideal_speed=2
                    R_control.ideal_speed=2
                    print("front")
                elif tagid == 3:
                    L_control.ideal_speed=0
                    R_control.ideal_speed=0
                    L_control.pre_duty=0
                    R_control.pre_duty=0
                    print("stop")
                    time.sleep(1)
    cap.release()  # 释放摄像头
    cv2.destroyAllWindows()  # 关闭所有显示窗体    

thread1=threading.Thread(target=scanner)
thread1.start()
speed=2
l_origin_duty=3
r_origin_duty=5
pwma.start(l_origin_duty)
pwmb.start(r_origin_duty)

	
try:
    L_control = PID(10,0.01,5,speed,l_origin_duty)
    R_control = PID(10,0.01,5,speed,r_origin_duty)
    while True:
        pwma.ChangeDutyCycle(L_control.update(lspeed))
        pwmb.ChangeDutyCycle(R_control.update(rspeed))
        print ('tarleft: %.2f tarright: %.2f left: %.2f  right: %.2f lduty: %.2f rduty: %.2f'%(L_control.ideal_speed,R_control.ideal_speed,lspeed,rspeed,L_control.pre_duty,R_control.pre_duty))
        time.sleep(1)
except KeyboardInterrupt:
    pass
pwma.stop()
pwmb.stop()	

