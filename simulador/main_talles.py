import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import time
# plt.ion()
# plt.figure(1)

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
fig_car, ax_car = plt.subplots()
ax.grid(True)

erroP = 0
erroSum = 0
erro = 0
vel_ref = 0.8
kp = 1.83 # 0.131
ki = 0.00
kd = 0.00749
Ts = 0.05

car = cp.CarCoppelia()
car.startMission()

def calculate_steering_angle(x, y, psi_car, psi_tra, x_ref, y_ref, v, k):
	erro_psi = psi_tra - psi_car
	mag_erro_pos = math.sqrt((x_ref - x)*(x_ref - x) + (y_ref - y)*(y_ref - y))
	ang_erro_pos = math.atan2((y_ref - y), (x_ref - x))
	ang_erro_tot = psi_car - ang_erro_pos
	ang_erro_tot = np.unwrap([ang_erro_tot])
	vet_erro_pos = (math.sin(ang_erro_tot[0])*mag_erro_pos)
	erro_pos = math.atan2(k*vet_erro_pos,v)
	delta = erro_psi + erro_pos
	
	return delta

while car.t < 10.0:
	car.step()

	vel = car.getVel()[0]
	erro = vel_ref - float(vel)
	erroSum = erroSum + erro*Ts
	c_kp = kp * erro
	c_ki = ki * erroSum
	c_kd = kd * (erro - erroP) / Ts
	acel = c_kp + c_ki + c_kd

	erroP = erro

	# atua
	car.setU(acel)

	# controlador lateral
	yaw = car.getYaw()
	x = car.getPos()[0]
	y = car.getPos()[1]
	v = car.getVel()[0]
	psi_tra = yaw
	k = 1
	x_ref = 0
	y_ref = y + abs(v) * Ts 

	delta = calculate_steering_angle(x , y, yaw, psi_tra, x_ref, y_ref, abs(v), k)
	car.setSteer(delta)

	image = car.getImage()

	# new_image = np.zeros(np.array(image).shape, dtype=np.uint8)
	# for x in range(600):
	# 	for y in range(400):
	# 		if image[y,x,0] < 60 and image[y,x,0] > 50:
	# 			new_image[y,x,:] = (0,0,0)
 
	image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	image_blur = cv2.GaussianBlur(image_gray, (5,5), 0)

	edges = cv2.Canny(image_blur, 50, 155)
	# lines = cv2.HoughLines( edges[0:300,50:550], 
	lines = cv2.HoughLines( edges, 
							1, 
							np.pi/180, 
							threshold=50, 
							min_theta=np.radians(-50), 
							max_theta=np.radians(50))  

	image_with_lines = image.copy()
	
	# # fig.clf()
	if lines is not None:
		for rho, theta in lines[:, 0]:
			
			if theta > np.pi/2:
				continue
			
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a * rho
			y0 = b * rho
			x1 = int(x0 + 1000 * (-b))
			y1 = int(y0 + 1000 * (a))
			x2 = int(x0 - 1000 * (-b))
			y2 = int(y0 - 1000 * (a))
			cv2.line(image_with_lines, (x1, y1), (x2, y2), (255, 0, 0), 2)  

			ax.plot([theta], [rho], 'bo')


	# plt.clf()
	# plt.gca().imshow(image_with_lines, origin='lower', cmap='gray')
	# plt.axis('off')

	fig_car.clf()
	fig_car.gca().imshow(image_with_lines, origin='lower', cmap='gray')
	ax_car.axis('off')
 
	plt.show()
	plt.pause(0.01)

	time.sleep(1000)
	break

car.stopMission()
	
print('Terminou...')