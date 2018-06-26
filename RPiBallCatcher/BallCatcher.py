# -*- coding: utf-8 -*-
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from fractions import Fraction
from threading import Thread, Event
import numpy as np
import time
import cv2
import serial

globalEvent = Event()

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
	def __init__(self, TamMax_x = 640, TamMax_y = 480, framerate=60):
		# initialize the camera and stream
		resolution = (TamMax_x, TamMax_y)
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.camera.exposure_mode = 'sports'
		self.camera.awb_mode = 'off'
		self.camera.awb_gains = (Fraction(95, 64), Fraction(311, 256))
		#g = self.camera.awb_gains
		#self.camera.awb_mode = 'off'
		#self.camera.awb_gains = g
		#self.camera.shutter_speed = 3000

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
		j = 0
		for f in self.stream:
			self.unread = True
			#print i
			j += 1
			tic = time.time()
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			#cv2.imwrite("frame"+"{0:03d}".format(j)+"_0.jpg",self.frame)
			'''if i>200:
				exit()'''
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return
			toc = time.time()
			#print "Elapsed time ", toc-tic

	def read(self):
		# return the frame most recently read
		if self.unread:
			self.unread = False
			return True,self.frame
		else:
			return False,self.frame
 
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True



class BallCatcherMain:
	def __init__(self, TamMax_x = 640, TamMax_y = 480, framerate = 60):
		self.actual = "reconocimiento"
		self.video = PiVideoStream(TamMax_x,TamMax_y, framerate)
		self.TamMax_x = TamMax_x
		self.TamMax_y = TamMax_y
		self.mouseON = False
		self.calculateHSV = False
		self.coords_colP = [(0,0),(1,1)]
		self.rect = []
		#self.HMinP = 0
		#self.SMinP = 0
		#self.VMinP = 0
		#self.HMaxP = 255
		#self.SMaxP = 255
		#self.VMaxP = 255
		#self.HMinC = 0
		#self.SMinC = 0
		#self.VMinC = 0
		#self.HMaxC = 255
		#self.SMaxC = 255
		#self.VMaxC = 255
		
		self.HMaxP = int(101)#int(25)
		#self.HMaxP = int(14)
		self.SMaxP = int(255)
		self.VMaxP = int(255)
		#self.HMinP = int(7)
		self.HMinP = int(60)#int(1)
		self.SMinP = int(120)#int(min(S_P))
		self.VMinP = int(80)
		print self.HMaxP,self.HMinP,self.SMaxP,self.SMinP,self.VMaxP,self.VMinP
		self.HMaxC = int(180)
		self.SMaxC = int(255)
		self.VMaxC = int(255)
		self.HMinC = int(170)
		self.SMinC = int(80)
		self.VMinC = int(0)
		
		self.contP = []
		self.contC = []
		self.AreaTotalP = []
		self.AreaTotalC = []
		self.ser = serial.Serial("/dev/ttyS0", baudrate = 9600,timeout = 0)

		############
		self.pposx = 0
		self.pposy = 0
		self.cposx = 0
		self.cposy = 0
		###########
		self.show = True # para mostrar imagen
		self.text = True # Para mostrar altura
		self.altura = 0
		###########
		self.numero = 0
		###########
		self.tiempo_inicial = time.time()
		########## Variables necesarias para predictivo
		self.largo = 10
		self.datos_x = np.zeros(self.largo)
		self.datos_y = np.zeros(self.largo)
		self.datos_z = np.zeros(self.largo)
		self.datos_t = np.zeros(self.largo)
		self.indice = 0
		self.begin_time = True
		self.altura_canasta = 10 #consideramos altura de canasta en 10 cm
		self.cxp = -1
		self.cyp = -1
		self.cxpp = -1
		self.cypp = -1
		############ Para guardar datos
		self.file_number = 0
		self.radio = 0
		##### Valores calibracion base
		self.base_x0 = int(108)
		self.base_y0 = int(81)
		self.base_x1 = int(339)
		self.base_y1 = int(245)


            
	def initialize_window(self):
		cv2.namedWindow("Ball Catcher")
		#cv2.setMouseCallback("Ball Catcher", self.selectROI)

	def inicio(self):          
		self.vid = self.video.start()  # Inicia el Thread de captura de video
		self.initialize_window()
		time.sleep(2)		# Permite que la camara se inicialice
		self.run()
		self.update()

	def update(self):
		self.index = 0
		while True:
			self.newImage,self.image = self.vid.read()
						
			#if self.index >= 200:
			#	exit()
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
				
				
				if self.calculateHSV:
					img_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
					#blur = cv2.blur(img_hsv,(10,10))
					self.HSV_Roi(self.coords_colP, img_hsv)
					self.calculateHSV = False
					# Tiempo para esperar sacar la pelota despues de calibracion
					self.actual = "reconocimiento"
					time.sleep(1)

			elif self.actual == "reconocimiento":
				if self.newImage:
					tic = time.time()
					#cv2.imwrite("imagen"+"{0:03d}".format(self.index)+".jpg",self.image)
					self.index += 1
					img_hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
					# mask of different colors
					#blur = cv2.blur(img_hsv,(10,10))
					#maskp = cv2.inRange(blur, np.array([self.HMinP,self.SMinP,self.VMinP]), np.array([self.HMaxP,self.SMaxP,self.VMaxP]))
					maskp = cv2.inRange(img_hsv, np.array([self.HMinP,self.SMinP,self.VMinP]), np.array([self.HMaxP,self.SMaxP,self.VMaxP]))
					maskc = cv2.inRange(img_hsv, np.array([self.HMinC,self.SMinC,self.VMinC]), np.array([self.HMaxC,self.SMaxC,self.VMaxC]))
					maskTotal = maskp + maskc
	                            
					seg = cv2.bitwise_and(self.image,self.image,mask=maskTotal) ## Chequear esta linea
	
					_, self.contP,_ = cv2.findContours(maskp, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # todos los contornos sin orden de jerarquia, solo extremos de un segmento
					_, self.contC,_ = cv2.findContours(maskc, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	                    
					for k in self.contP:
						Mptemp = cv2.moments(k)
						areap = Mptemp['m00']
						self.AreaTotalP.append(areap)
	                    
					for k in range(0,len(self.contC)):
						Mctemp = cv2.moments(self.contC[k])
						areac = Mctemp['m00']
						self.AreaTotalC.append(areac)
	
					if self.AreaTotalP != []:
						indP = self.AreaTotalP.index(max(self.AreaTotalP))
						(xP,yP), self.radio = cv2.minEnclosingCircle(self.contP[indP])
						centerP = (int(xP),int(yP))
	                        
						if self.show:
							cv2.circle(seg, centerP, int(self.radio), (0,255,0))
						areaPelota = 3.14*(self.radio**2)
						if areaPelota != 0:
							self.cxp = int(xP)
							self.cyp = int(yP)
	                            
						else:
							self.cxp = -1
							self.cyp = -1
					else:
						self.cxp = -1
						self.cyp = -1
	
	                    
					if self.AreaTotalC != []:
						indC = self.AreaTotalC.index(max(self.AreaTotalC))
						xC,yC,w,h = cv2.boundingRect(self.contC[indC])
						if self.show:
							cv2.rectangle(seg, (xC,yC), (xC+w,yC+h), (255,0,0))
						if w != 0:
							cxc = xC+(w/2)
							cyc = yC+(h/2)
						else:
							cxc = 0
							cyc = 0
					else:
						cxc = 0
						cyc = 0
	
					#### Lineas para calcular la altura de la pelota        
					if self.radio > 1:
						self.altura = 145 - 1748.91421664882/(self.radio) ## base 145
						## self.altura = 145 - 950/(self.radio)  ## Low Values
						#self.altura = 168 - 1092/(self.radio)  ## High Values
						#self.altura = 168 - 1698/self.radio  ## High Resolution
	                    
					if self.text:                        
						cv2.putText(seg, str(self.altura)+'cm', (5,self.TamMax_y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
					#self.imagen_mostrar = self.image
					self.imagen_mostrar = seg
	
					# escalamiento de datos para envio por serial
					if self.cxpp < self.base_x0+1:
						self.pposx = int(127)
					elif self.cxpp < self.base_x1+1:
						self.pposx = int(254*(self.cxpp-self.base_x0)/(self.base_x1 - self.base_x0))
					else:
						self.pposx = int(127)
	
					if self.cypp < self.base_y0+1:
						self.pposy = int(127)
					elif self.cypp < self.base_y1+1:
						self.pposy = 254-int(254*(self.cypp-self.base_y0)/(self.base_y1-self.base_y0))
					else:
						self.pposy = int(127)
	               
					if cxc < self.base_x0+1:
						self.cposx = int(127)
					elif cxc < self.base_x1+1:
						self.cposx = int(254*(cxc-self.base_x0)/abs(self.base_x0 - self.base_x1))
					else:
						self.cposx = int(127)
	
					if cyc < self.base_y0+1:
						self.cposy = int(127)
					elif cyc < self.base_y1+1:
						self.cposy = 254-int(254*(cyc-self.base_y0)/abs(self.base_y0-self.base_y1))
					else:
						self.cposy = int(127)
	    
					self.ser.write(chr(255))
					self.ser.write(chr(self.pposx))
					self.ser.write(chr(self.pposy))
					self.ser.write(chr(self.cposx))
					self.ser.write(chr(self.cposy))
	
					# desplegar el circulo de la posicion predicha
					if 0 < self.cxpp and self.cxpp < self.TamMax_x:
						if 0 < self.cypp and self.cypp < self.TamMax_y:
							cv2.circle(self.imagen_mostrar, (self.cxpp,self.cypp), 4, (0,0,255), -1) ## Dibuja punto ROJO
					# despliega el circulo de la posicion de la pelota y canasta
					cv2.circle(self.imagen_mostrar, (self.cxp,self.cyp), 3, (0,255,0), -1)
					cv2.circle(self.imagen_mostrar, (cxc,cyc), 3, (255,0,0), -1)
					# cv2.circle(self.imagen_mostrar, (int(self.TamMax_x/2),int(self.TamMax_y/2)), 3, (255,0,255), -1)
					self.newImage = False
					globalEvent.set()

			## debug para cerrar programa
			key = cv2.waitKey(1) & 0xFF
			if key == 27: #esc
				g = self.video.camera.awb_gains
				print g
				self.vid.stop()
				break
						
			#elif key == 99: # c
				#self.actual = "calibration"
			#elif key == 100: # d
				#print self.radio
			#elif key == 115: # s
				#if self.show:
					#self.show = False
				#else:
					#self.show = True
			#elif key == 116: # t
				#if self.text:
					#self.text = False
				#else:
					#self.text = True
			#elif key ==  114: # r
				#self.datos_x = np.zeros(self.largo)
				#self.datos_y = np.zeros(self.largo)
				#self.datos_z = np.zeros(self.largo)
				#self.datos_t = np.zeros(self.largo)
				#self.indice = 0
				#self.begin_time = True
				#self.cxpp = -1
				#self.cypp = -1
                    
               
			if self.show:     
				cv2.imshow("Ball Catcher", self.imagen_mostrar)
				#if punto:
					#self.show = False
    ##                print time.time() - self.tiempo_actual
			self.AreaTotalP = []
			self.AreaTotalC = []
    ##                print 'total ', time.time() - self.tiempo_inicial_1


	def run(self):
	# Inicia thread de captura de informacion
		t = Thread(target=self.predictivo, args=())
		t.daemon = True
		t.start()
		return self


	def predictivo(self):
		self.tiempo_inicial_1 = time.time()
		k = -981/2
		while True:
			globalEvent.wait()
			#cv2.imwrite("imagen"+str(self.indice)+".jpg",self.image)
            ############## Desde esta linea comenzará el predictivo
            ############## Pensar idea de paralelizar
			if self.cxp > 0 and self.cyp > 0:  # Esta condicion es para saber de que entró la pelota al campo de visión
				'''
				Se van guardando los datos con cada iteracion
				'''
				
								#	if self.indice > 0 and self.datos_z[self.indice] == self.datos_z[self.indice - 1]:
										
				if self.altura > 0:
					self.datos_x[self.indice] = self.cxp
					self.datos_y[self.indice] = self.cyp
					self.datos_z[self.indice] = self.altura - self.altura_canasta
					self.datos_t[self.indice] = time.time() - self.tiempo_inicial_1
                        
                    
					if self.indice >= 1:
						kt = k*np.power(self.datos_t[0:self.indice + 1],2)
						z_mov = np.polyfit(self.datos_t[0:self.indice + 1], self.datos_z[0:self.indice + 1]-kt,1)
						p_mov = np.poly1d(np.concatenate((np.array([k]),z_mov)))
						#z_mov = np.polyfit(self.datos_t[0:self.indice + 1], self.datos_z[0:self.indice + 1],2)
						#p_mov = np.poly1d(z_mov)
						t_caida = max(p_mov.r)
						'''print "tiempo: ",self.indice, ' ' ,self.datos_t[1:self.indice + 1]
						print "altura: ",self.indice, ' ' ,self.datos_z[1:self.indice + 1]
						print "coefic: ", self.indice, ' ', p_mov
						print "tfinal: ", self.indice, ' ', t_caida
						print 'fit z ', time.time() - self.tiempo_inicial_1'''

						## Velocidad en x
						x_mov = np.polyfit(self.datos_t[0:self.indice + 1], self.datos_x[0:self.indice + 1],1)
						x_fin = np.poly1d(x_mov)
                            
						self.cxpp = int(round(x_fin(t_caida)))

						## Velocidad en y
						y_mov = np.polyfit(self.datos_t[0:self.indice + 1], self.datos_y[0:self.indice + 1],1)
						y_fin = np.poly1d(y_mov)
						self.cypp = int(round(y_fin(t_caida)))
						

				else:
					print "@@@@@@@@@@@@@@@@@"

				# aumentamos el indice
				if self.altura > 0:
					self.indice += 1

				if self.indice > 8:
					#self.show = False
					self.file_number += 1 
					name_file = 'lanzamiento' + str(self.file_number) + '.txt'
					#with open(name_file,'w') as archivo:
						#archivo.write("t,x,y,z\n")
						#for i in range(self.largo):
							#archivo.write(str(self.datos_t[i])+','+str(self.datos_x[i])+','+str(self.datos_y[i])+','+str(self.datos_z[i])+'\n')
					print z_mov
					self.indice = 0
					self.datos_x = np.zeros(self.largo)
					self.datos_y = np.zeros(self.largo)
					self.datos_z = np.zeros(self.largo)
					self.datos_t = np.zeros(self.largo)
					time.sleep(2)
			globalEvent.clear()
                       

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
				## self.actual = 'reconocimiento'
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
	    
		self.HMaxP = int(101)#101
		#self.HMaxP = int(14)
		self.SMaxP = int(255)
		self.VMaxP = int(255)
		#self.HMinP = int(7)
		self.HMinP = int(60)#60
		self.SMinP = int(120)
		self.VMinP = int(80)
		print self.HMaxP,self.HMinP,self.SMaxP,self.SMinP,self.VMaxP,self.VMinP
		self.HMaxC = int(180)
		self.SMaxC = int(255)
		self.VMaxC = int(255)
		self.HMinC = int(170)
		self.SMinC = int(80)
		self.VMinC = int(0)


if __name__=='__main__':
	
	Tam_x = 480
	Tam_y = 320
	fps = 32
	main = BallCatcherMain(Tam_x,Tam_y,fps)
	main.inicio()
	cv2.destroyAllWindows()
