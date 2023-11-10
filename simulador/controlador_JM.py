# -*- coding: utf-8 -*-
# Disciplina: Tópicos em Engenharia de Controle e Automação IV (ENG075): 
# Fundamentos de Veículos Autônomos - 2023/2
# Professores: Armando Alves Neto e Leonardo A. Mozelli
# Cursos: Engenharia de Controle e Automação
# DELT – Escola de Engenharia
# Universidade Federal de Minas Gerais
########################################

import class_car as cp
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
# plt.ion()
# plt.figure(1)

erroP = 0
erroSum = 0
erro = 0
vel_ref = 0.8
kp = 1.83 # 0.131
ki = 0.00
kd = 0.00749
Ts = 0.05

########################################
# cria comunicação com o carrinho
car = cp. CarCoppelia()
car.startMission()

permissao = 0 #####################################################################################################################

def calculate_steering_angle(x, y, psi_car, psi_tra, x_ref, y_ref, v, k):
    # Calcule o erro lateral (cross-track error)
    # erro_psi = psi_tra - psi_car
    # erro_pos = math.atan2(k*math.sqrt((x_ref - x)*(x_ref - x) + (y_ref - y)*(y_ref - y)),v)
    # error = -(x - x_ref) * math.sin(theta) + (y - y_ref) * math.cos(theta)
    # delta = erro_psi + erro_pos

    # Calcule o ângulo de direção desejado 
    # delta = math.atan2(2 * error, v)

    # print('Erro: {:.2f}'.format(erro_pos))

    e = psi_tra - psi_car
    steer = np.arctan2(k*e, v)
    
    return steer

# def calculate_steering_angle(x, y, psi_car, psi_tra, x_ref, y_ref, v, k):
#     # Calcule o erro lateral (cross-track error)
#     erro_psi = psi_tra - psi_car
#     erro_pos = math.atan(k*math.sqrt((x_ref - x)*(x_ref - x) + (y_ref - y)*(y_ref - y))/v)

#     front_axle_vec = [-np.cos(psi_car + np.pi / 2), -np.sin(psi_car + np.pi / 2)]
#     error_front_axle = np.dot([x_ref, y_ref], front_axle_vec)

#     erro_pos = np.arctan2(k * error_front_axle, v)
#     print("Erro: ", error_front_axle, "Vel: ", v, "Erro_pos: ", erro_pos * 180 / math.pi)
#     # Calcule o ângulo de direção desejado 
#     delta = erro_psi + erro_pos

#     return delta


while car.t < 15.0:
	
	# lê sensores
	car.step()
	
	# seta direcao
	# delta = -2.0*(0.0 - car.p[0]) + 5.0*(0.0 - car.w)
	# car.setSteer(-delta)

	# controlador longitudinal PID
	# vel = sum(car.getVel())
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

	# if permissao == 1:
	# 	a = car.getImage # Cada matrix 3x3 é um pixel(?) e a imagem é de 400 x 600
	# 	b = a()
	# 	permissao = 2
	# 	plt.imshow(b, interpolation='nearest')
	# 	plt.show()


	# controlador lateral
	yaw = car.getYaw()
	# while yaw > np.pi:
	# 	yaw -= 2.0 * np.pi

	# while yaw < np.pi:
	# 	yaw += 2.0 * np.pi

								# x, y, psi_car, psi_tra, x_ref, y_ref, v, k
	x = car.getPos()[0]
	y = car.getPos()[1]
	v = car.getVel()[0]
	psi_tra = yaw
	k = 1
	x_ref = 0
	# y_ref = car.t
	y_ref = y + abs(v) * Ts

	steer = calculate_steering_angle(x , y, yaw, psi_tra, x_ref, y_ref, v, k)
	# print('x: {:.2f}'.format(x), 'y: {:.2f}'.format(y), '| {:.2f}'.format(car.t), 'yaw: {:.2f}'.format(yaw), '| {:.2f}'.format(psi_tra), 'delta: {:.2f}'.format(delta))
	print('steer: {:.2f}'.format(steer))
	car.setSteer(-steer)


	# # plota
	# plt.clf()
	# t = [traj['t'] for traj in car.traj]
	# v = [traj['v'] for traj in car.traj]
	# plt.plot(t,v)
	# plt.show()
	# plt.pause(0.01)

	# with open("dados.txt", "w") as arquivo:
    # # Escreve os dados no arquivo
	# 	for dadooo in t:
	# 		arquivo.write(str(dadooo))
	# 		arquivo.write('\n')

	# 	arquivo.write("---------------------------\n")

	# 	for dadoooo in v:
	# 		arquivo.write(str(dadoooo))
	# 		arquivo.write('\n')
	
print('Terminou...')