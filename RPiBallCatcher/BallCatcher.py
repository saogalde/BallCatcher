# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np
import time
import cv2
import serial

'''
Carga de librerias SPI
'''
#import spidev


## Version 1.
"""
9-06
Se le incluye soporte para SPI 
"""


class PiVideoStream:
    def __init__(self, TamMax_x = 640, TamMax_y = 480, framerate=32):
	# initialize the camera and stream
        resolution = (TamMax_x, TamMax_y)
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        ##############
        #self.camera.iso = 100
        #self.camera.shutter_speed = self.camera.exposure_speed
        #self.camera.exposure_mode = 'off'
        #g = self.camera.awb_gains
        #self.camera.awb_mode = 'off'
        #self.camera.awb_gains = g
        ##############
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
                        format="bgr", use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

    def start(self):
	# start the thread to read frames from the video stream
        t = Thread(target=self.update, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
	# keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                return

    def read(self):
	# return the frame most recently read
	return self.frame

    def stop(self):
	# indicate that the thread should be stopped
	self.stopped = True



class BallCatcherMain:
	def __init__(self, TamMax_x = 640, TamMax_y = 480):
            self.actual = "base"
            self.video = PiVideoStream()
            self.TamMax_x = TamMax_x
            self.TamMax_y = TamMax_y
            self.mouseON = False
            self.calculateHSV = False
            self.coords_colP = [(0,0),(1,1)]
            self.rect = []
            self.HMinP = 0
            self.SMinP = 0
            self.VMinP = 0
            self.HMaxP = 255
            self.SMaxP = 255
            self.VMaxP = 255
            self.HMinC = 0
            self.SMinC = 0
            self.VMinC = 0
            self.HMaxC = 255
            self.SMaxC = 255
            self.VMaxC = 255
            self.contP = []
            self.contC = []
            self.AreaTotalP = []
            self.AreaTotalC = []
            self.ser = serial.Serial("/dev/ttyS0", baudrate = 9600,timeout = 0)
##            '''
##            SPI LINEAS NUEVAS
##            MOSI -> GPIO10 (19)                                                                                     
##            MISO -> GPIO09 (21)
##            SCK -> GPIO11  (23)
##            CS tiene 2
##            CE1 -> GPIO07  (26)
##            CE0 -> GPIO08  (24)
##            '''
##            self.spi = spidev.SpiDev()
            self.radius = []

	def initialize_window(self):
	    cv2.namedWindow("Ball Catcher")
	    cv2.setMouseCallback("Ball Catcher", self.selectROI)

	def inicio(self):
          
	    self.vid = self.video.start()
	    self.initialize_window()
            #self.video.start() # Inicia el Thread de captura de video
	    time.sleep(2)		# Permite que la camara se inicialice
	    # self.run()
	    self.update()

	def update(self):
            while True:
                self.image = self.vid.read()
                if self.actual == "base":
                    if self.mouseON:
                        cv2.rectangle(self.image, self.coords_colP[0], self.rect[0], (0, 255, 0), 2)
                    self.imagen_mostrar = self.image
                    #cv2.imshow("Ball Catcher", self.imagen_mostrar)
                    
                elif self.actual == "calibration":

                    if self.mouseON:
                        cv2.rectangle(self.image, self.coords_colP[0], self.rect[0], (0, 255, 0), 2)
                    self.imagen_mostrar = self.image
                    #cv2.imshow("Ball Catcher", self.imagen_mostrar)

                elif self.actual == "reconocimiento":
                    img_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
                    if self.calculateHSV:
                        #blur = cv2.GaussianBlur(img_hsv,(15,15),0)
                        self.HSV_Roi(self.coords_colP, img_hsv)
                        self.calculateHSV = False
                        
                    # mask of different colors
                    #blur = cv2.blur(img_hsv,(10,10))
                    #maskp = cv2.inRange(blur, np.array([self.HMinP,self.SMinP,self.VMinP]), np.array([self.HMaxP,self.SMaxP,self.VMaxP]))
                    #maskc = cv2.inRange(blur, np.array([self.HMinC,self.SMinC,self.VMinC]), np.array([self.HMaxC,self.SMaxC,self.VMaxC]))
                    maskp = cv2.inRange(img_hsv, np.array([self.HMinP,self.SMinP,self.VMinP]), np.array([self.HMaxP,self.SMaxP,self.VMaxP]))
                    maskc = cv2.inRange(img_hsv, np.array([self.HMinC,self.SMinC,self.VMinC]), np.array([self.HMaxC,self.SMaxC,self.VMaxC]))
                    
                    # Noise Filter (erosion and dilation)
                    #kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(5,5))
                    #openingP = cv2.morphologyEx(maskp, cv2.MORPH_OPEN, kernel)
                    #openingC = cv2.morphologyEx(maskc, cv2.MORPH_OPEN, kernel)
                    #maskTotal = openingP + openingC
                    maskTotal = maskp + maskc
                            
                    seg = cv2.bitwise_and(self.image,self.image,mask=maskTotal) ## Chequear esta linea

                    #_, self.contP,_ = cv2.findContours(openingP, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # todos los contornos sin orden de jerarquia, solo extremos de un segmento
                    #_, self.contC,_ = cv2.findContours(openingC, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # todos los contornos sin orden de jerarquia, solo extremos de un segmento

                    _, self.contP,_ = cv2.findContours(maskp, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # todos los contornos sin orden de jerarquia, solo extremos de un segmento
                    _, self.contC,_ = cv2.findContours(maskc, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    
                    for k in range(0,len(self.contP)):
                        Mptemp = cv2.moments(self.contP[k])
                        areap = Mptemp['m00']
                        self.AreaTotalP.append(areap)
                    
                    #print len(self.contP)
                    for k in range(0,len(self.contC)):
                        Mctemp = cv2.moments(self.contC[k])
                        areac = Mctemp['m00']
                        self.AreaTotalC.append(areac)

                    if self.AreaTotalP != []:
                        
                        indP = self.AreaTotalP.index(max(self.AreaTotalP))
                        (xP,yP), radioP = cv2.minEnclosingCircle(self.contP[indP])
                        centerP = (int(xP),int(yP))
                        radioP = int(radioP)
                        cv2.circle(seg, centerP, radioP, (0,255,0))
                        self.Mp = cv2.moments(self.contP[indP])
                        aP = 3.14*(radioP**2)
                        if self.Mp['m00'] != 0: 
##                            cxp = int(self.Mp['m10']/self.Mp['m00'])
##                            cyp = int(self.Mp['m01']/self.Mp['m00'])
                            cxp = int(xP)
                            cyp = int(yP)
                        else:
                            cxp = 0
                            cyp = 0
                    else:
                        cxp = 0
                        cyp = 0

                    
                    if self.AreaTotalC != []:
                        indC = self.AreaTotalC.index(max(self.AreaTotalC))
                        xC,yC,w,h = cv2.boundingRect(self.contC[indC])
                        cv2.rectangle(seg, (xC,yC), (xC+w,yC+h), (255,0,0))
                        Mc = cv2.moments(self.contC[indC])
                        if Mc['m00'] != 0: 
##                            cxc = int(Mc['m10']/Mc['m00'])
##                            cyc = int(Mc['m01']/Mc['m00'])
                            cxc = xC+(w/2)
                            cyc = yC+(h/2)
                        else:
                            cxc = 0
                            cyc = 0
                    else:
                        cxc = 0
                        cyc = 0
                            
                        
                    
##                    # calculate mass center of the segmented object
##                    Mp=cv2.moments(openingP)
##                    Mc=cv2.moments(openingC)
##                    
##                    if Mp['m00'] != 0: 
##                        cxp = int(Mp['m10']/Mp['m00'])
##                        cyp = int(Mp['m01']/Mp['m00'])
##                    else:
##                        cxp = 0
##                        cyp = 0
##
##                    if Mc['m00'] != 0: 
##                        cxc = int(Mc['m10']/Mc['m00'])
##                        cyc = int(Mc['m01']/Mc['m00'])
##                    else:
##                        cxc = 0
##                        cyc = 0
                        
                    #print'Pelota real:',cxp,cyp
                    #proceso de normalizacion de eje x y eje y
##                    pposx = int(254*cxp/self.TamMax_x)
##                    pposy = int(254*cyp/self.TamMax_y)
##
##                    cposx = int(254*cxc/self.TamMax_x)
##                    cposy = int(254*cyc/self.TamMax_y)
                    

                    if cxp < self.base_x0+1:
                        pposx = 0
                    elif cxp < self.base_x1+1:
                        pposx = int(254*(cxp-self.base_x0)/(self.base_x1 - self.base_x0))
                        if pposx < 0:
                            pposx =  0
                    else:
                        pposx = 254

                    if cyp < self.base_y0+1:
                        pposy = 254
                    elif cyp < self.base_y1+1:
                        pposy = 254-int(254*(cyp-self.base_y0)/(self.base_y1-self.base_y0))
                        if pposy < 0:
                            pposy =  0
                    else:
                        pposy = 0
               

                    if cxc < self.base_x0+1:
                        cposx = 0
                    elif cxc < self.base_x1+1:
                        cposx = int(254*(cxc-self.base_x0)/abs(self.base_x0 - self.base_x1))
                        if cposx < 0:
                            cposx =  0
                    else:
                        cposx = 254

                    if cyc < self.base_y0+1:
                        cposy = 254
                    elif cyp < self.base_y1+1:
                        cposy = 254-int(254*(cyc-self.base_y0)/abs(self.base_y0-self.base_y1))
                        if cposy < 0:
                            cposy =  0
                    else:
                        cposy = 0
##                    print'P (',pposx,pposy,'),C (',cposx,cposy,')','A ',aP
                     

                    self.ser.write(chr(255))
                    self.ser.write(chr(pposx))
                    self.ser.write(chr(pposy))
                    self.ser.write(chr(cposx))
                    self.ser.write(chr(cposy))

##                    '''
##                    Nuevas lineas para comunicacion SPI
##                    Se abre el puerto, se mandan los datos y se cierra el puerto
##                    Deben enviarse como hexadecimal
##                    '''
##                    self.spi.open(0,0)
##                    ## esto es para que podamos ocupar estos valores por otro lado
##                    self.send = [255,pposx,pposy,cposx,cposy]
##                    resp = self.spi.xfer(self.send)
##                    self.spi.close()

                    cv2.circle(seg, (cxp,cyp), 3, (0,255,0))
                    cv2.circle(seg, (cxc,cyc), 3, (255,0,0))

                    if radioP < 2:
                        altura = 170
                    else:
                        altura = 170 - 4*548.57/(radioP*2)
                    
                    cv2.putText(seg, str(altura)+' cm', (5,465), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                    self.imagen_mostrar = seg

                ## debug para cerrar programa
                key = cv2.waitKey(1) & 0xFF
                if key == 27: #esc
                ## 	No estoy seguro si esto detendra los threads, hay que chequear
                    self.vid.stop()
                    break
                    
                elif key == 99: # c
                    self.actual = "calibration"
                elif key == 100:
                    self.radius.append((centerP, radioP))
                    print self.radius
                
                    
                cv2.imshow("Ball Catcher", self.imagen_mostrar)
                self.AreaTotalP = []
                self.AreaTotalC = []



##	def run(self):
##	# Inicia thread de captura de informacion
##	    t = Thread(target=self.update, args=())
##	    t.daemon = True
##	    t.start()
##	    return self



        def selectROI(self, event, x, y, flags, param):
            """
            Selecciona con el mouse el area de la region de interes
            event = raton click event
            x = coordenada x
            y = coordenada y
            flags = parametro que entrega la funcion setMouseCallback
            param = parametro que entrega la funcion setMouseCallback 
            """
            if event == cv2.EVENT_LBUTTONDOWN:
                self.coords_colP = [(x,y)]
                self.rect = [(x,y)]
                self.mouseON = True
                print'Se apreto Mouse', self.coords_colP
            if event == cv2.EVENT_MOUSEMOVE and self.mouseON == True:
                self.rect = [(x,y)]
            if event == cv2.EVENT_LBUTTONUP:
                self.coords_colP.append((x,y))
                self.mouseON = False
                if self.actual == 'base':
                    self.base_x0 = int(self.coords_colP[0][0])
                    self.base_y0 = int(self.coords_colP[0][1])
                    self.base_x1 = int(self.coords_colP[1][0])
                    self.base_y1 = int(self.coords_colP[1][1])
                    self.actual = 'calibration'
                    print'(',self.base_x0,self.base_y0,')','(',self.base_x1,self.base_y1,')'
                elif self.actual == 'calibration':
                    self.actual = 'reconocimiento'
                    self.calculateHSV = True
                print'Se solto Mouse', self.coords_colP

	def HSV_Roi(self, p_element1, HSV_image):
            """
            Obtiene valores HSV de la region de interes
            p_element1 = Lista con tuplas de coordenadas superior izquierda, inferior derecha
            HSV_iamge = imagen completa en HSV -> Matriz 
            """
            H_P = []
            S_P = []
            V_P = []
	    for i in range(p_element1[0][1],p_element1[1][1]):
	        for j in range(p_element1[0][0],p_element1[1][0]):
	            H_P.append(HSV_image[i][j][0])
	            if HSV_image[i][j][0] == 0:
	                H_P.remove(0)
##	            S_P.append(HSV_image[i][j][1])
##	            V_P.append(HSV_image[i][j][2])
##	            
	    
	    self.HMaxP = int(max(H_P))#int(25)
            #self.HMaxP = int(14)
	    self.SMaxP = int(255)
	    self.VMaxP = int(255)
	    #self.HMinP = int(7)
	    self.HMinP = int(min(H_P))#int(1)
	    self.SMinP = int(200)#int(min(S_P))
	    self.VMinP = int(100)
	    print self.HMaxP,self.HMinP,self.SMaxP,self.SMinP,self.VMaxP,self.VMinP
	    self.HMaxC = int(180)
	    self.SMaxC = int(255)
	    self.VMaxC = int(255)
	    self.HMinC = int(170)
	    self.SMinC = int(80)
	    self.VMinC = int(0)





if __name__=='__main__':

	main = BallCatcherMain()
	main.inicio()
	cv2.destroyAllWindows()
