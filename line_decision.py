#!/usr/bin/python
# -*- coding: utf-8 -*-


import rospy
import time as tp
from image_processing.msg import drive_param				#parametros desejados de frenagem e aceleracao do veiculo
from image_processing.msg import coords						#leitura de coordenadas do carro
import math								
import numpy as np


prev_theta= 0.0
dt= 0
count= 0
var_print= False
ZERO= 0


MIN_SLOPE= 1.0
MAX_ERRO_INTEGRADOR= 2.5
MAX_STEER = 30.0				#maxima angulacao da roda em graus
MIN_STEER = -30.0				#minima angulacao da roda em graus


ANG_TO_RAD	= np.pi/180			#converte graus para radiano
RAD_TO_ANGLE = 180/np.pi		#converte radiano para graus

KMH_TO_MS = 1/3.6				#converte de Km/H para m/s
MS_TO_KMH = 3.6	

msg = drive_param()
msg.velocity= 0.0
msg.angle= 0.0

pub= 0

#PID das rodas
kp_steer_l=3.3					#Kp 40 km - 3.2			kp 50 km 5.5  5				18		18
ki_steer_l= 2				#Ki 40 km - 0.01		Ki 50 km 0.01   0			0			3
kd_steer_l= 1.5				#Kd 40 km - 0.2			Kd 50 km 0.4  0.45			7			1

kp_steer_c= 3.3				#Kp 40 km - 4.0			Kp 50 km 5.5			18			18
ki_steer_c= 1.8				#Ki 40 km - 0.005		Ki 50 km 0.075				0			3
kd_steer_c= 1.3						#Kd 40 km - 0.25		Kd 50 km 0.25			7			1

steer_integral = 0.0
steer_deriv = 0.0
theta_error = 0.0


#-----------------------------------------------------------------------
#FUNCAO: calc_PID(data)
#	Executa os comandos do PID
#	Retorna o erro da variavel de controle
# FUNCAO INUTILIZADA UMA VEZ QUE O CONTROLO E FEITO NO DECORRER DO CODIGO, ESTA FUNCAO NAO E CHAMADA------------------------------------------------------
#-----------------------------------------------------------------------
def calc_PID( theta_error, dt, slope, x2, x1):
	global steer_integral
	global prev_theta
	global steer_deriv
	#if abs(steer_integral) >= MAX_ERRO_INTEGRADOR or abs(theta_error) <= 0.1:			#zerando erro do integrador
	if abs(theta_error) <= 0.1:			#zerando erro do integrador
		steer_integral = ZERO
	elif abs(steer_integral) >= MAX_ERRO_INTEGRADOR:
		steer_integral = MAX_ERRO_INTEGRADOR
	else:	
		steer_integral = steer_integral + theta_error * dt								#increementando o integrador
	steer_deriv = prev_theta - theta_error

	if abs(slope) <= MIN_SLOPE and x2-x1 > 30:											#definindo uma acao de controle para curva ou reta
		control_error_steer = - (kp_steer_c*theta_error + ki_steer_c * steer_integral + kd_steer_c * steer_deriv)
		#print 'CURVA: ', slope
	else:
		control_error_steer = - (kp_steer_l*theta_error + ki_steer_l * steer_integral + kd_steer_l * steer_deriv)
		#print 'RETA: ', slope
	
	return control_error_steer





def vehicle_control (data):
	global time1
	global angle
	global velocity
	global msg
	global count
	global dt
	global prev_theta


#define o dt--------------------------------------------------------------------------------------------------------------------------------------
	time_now = tp.time()									#armazena o tempo atual
	if (count == 0):
		dt = 0.0											#inicializa a variavel
	else:	
		dt = time_now - time1								#tempo entre iteracoes
	time1 = time_now										#armazena o tempo atual para a proxima iteracao

	#armazenando os dados lidos do topico--------------------------------------------------------------------------------------------------------------------------------------
	x2 = data.X2												#posicao X2 da linha detectada
	y2 = data.Y2												#posicao Y2 da linha detectada--
	x1 = data.X1
	x_mean = (x2+x1)/2
	#x_mean = x2
	slope = data.slope
	if 1:
		print "x2: ", x2
		print "x1: ", x1
		print "x_mean: ", x_mean
		print "slope: ", slope

#basicamente inutil, ou seja , se estivesse mais ou menos alinhado nao recalcula? depois qd saia deste range ajustava demasiado. A remover
#	if x_mean < 565 and x_mean > 500:
		#definindo o erro de posicao do veiculo em relacao a linha 
#		error_steer = 0							#posicao X da linha em relacao ao centro da imagem 800 pixels
		#error_steer = x_mean-400								#posicao X da linha em relacao ao centro da imagem 800 pixels
#		if var_print:
#			print "Error: ",error_steer
#	else:
	if 1:
		error_steer = (x_mean-610)/610
		print "Sup yo \n",error_steer								#posicao X da linha em relacao ao centro da imagem 800 pixels----> Se não existir simplesmente para? Nao há nada alternativa? also: precisamente 2000?


#Ver se existe uma linha para seguir ou não
	if (x2 == 2000):
		velocity= 0.0
		angle= 30.0
		
	else:
		velocity = 4.3
		#chamar função que trata do ângulo: envia a posição X1 atual e onde quer estar X2, com respetiva inclinaçao e diferença de direção "error steer"
		angle= -(steering_control(x2, x1, velocity, error_steer, slope))
		
	msg.velocity= velocity
	msg.angle= angle
	pub.publish(msg)	




def steering_control(x2, x1, velocity, error_steer, slope):
	global ZERO
	global kp_steer_l						#Kp 40 km - 3.2			kp 50 km 5.5  5				18		18
	global ki_steer_l				#Ki 40 km - 0.01		Ki 50 km 0.01   0			0			3
	global kd_steer_l						#Kd 40 km - 0.2			Kd 50 km 0.4  0.45			7			1
	global kp_steer_c 						#Kp 40 km - 4.0			Kp 50 km 5.5			18			18
	global ki_steer_c					#Ki 40 km - 0.005		Ki 50 km 0.075				0			3
	global kd_steer_c						#Kd 40 km - 0.25		Kd 50 km 0.25			7			1
	global steer_integral
	global steer_deriv
	global msg
	global theta_error

#Controle de tempo de resposta---------------------------------------------CONTROLO DE VELOCIDADE PODE VIR AINDA A SER FEITO--------------------------------------------
#Entao.. as duas condições vao dar ao mesmo calculo? -----------------<<<<<<<
#De onde vem o 0.92 da friccao?
# distancia percorrida = velocidade? :) calculo que o carro nao ande 4,5m/s -------------------------------------------------------------------------------------------
	if abs(slope) <= MIN_SLOPE and x2-x1 > 25:
		dist_corr = velocity * 0.85
		print "\n Variavel: dist_corr nº1:",dist_corr
	else:
		dist_corr = velocity * 	0.85					#distancia a ser percorrida em metros em 1 s
		print "\n Variavel: dist_corr: n",dist_corr
	if var_print:
		print "\n Variavel: dist_corr:",dist_corr
		
#define o erro em termos de angulo--------------------------------------------------------------------------------------------------------------------------------------
	if (velocity != ZERO):
		dist_ang = error_steer/dist_corr						#angulo do erro em relacao a reta detectada    O else d necessario pa alguma coisa?
	else:
		dist_ang = ZERO
	#print "cat op sobre hip: ", error_steer/dist_corr
	if var_print:
		print "\n Variavel: dist_ang:",dist_ang

	#if(abs(dist_ang)<=1 and velocity != ZERO):
#		if var_print:
#			print "Correcao Normal"
	theta_error = np.arcsin(error_steer/dist_corr) * RAD_TO_ANGLE	#correcao normal
	#elif (velocity != ZERO):
	#	theta_error = np.arcsin((error_steer/abs(error_steer))/(dist_corr/abs(dist_corr))) * RAD_TO_ANGLE	#se a distancia inicial for pequena para o erro
	#	if var_print:
	#		print "Distancia Pequena"
	#else:
	#	theta_error = ZERO					#caso nao exista uma linha detectada
	
	if var_print:
		print "\n Variavel: theta_error: ", theta_error

	#Calculando o PID--------------------------------------------------------------------------------------------------------------------------------------
	if abs(steer_integral) >= MAX_ERRO_INTEGRADOR or abs(theta_error) <= 0.1:		#se o integrador comacar a crescer demais, zera
		steer_integral = ZERO
	else:
		steer_integral = steer_integral + theta_error * dt							#incrementa o integrador
	if var_print:
		print "steer_integral: ", steer_integral

	steer_deriv = prev_theta - theta_error											#calcula a diferenca do erro
	#print "previous error:", (prev_theta - theta_error)




	#sistema de controle de angulacao
	#Faz o trabalho da função PID, que por alguma razão nao é usada.
	#print '---------------------------------'
	if abs(slope) <= MIN_SLOPE and x2-x1 > 25:
		control_error_steer = -(kp_steer_c*theta_error + ki_steer_c * steer_integral + kd_steer_c * steer_deriv )
		print 'theta_error: ',theta_error
		#print 'steer_integral', steer_integral
		#print 'steer_deriv', steer_deriv
		print 'CURVA LALALALALALAAL: ', slope
		print 'CURVA: X2 - X1', x2-x1
		if var_print:
			print 'CURVA: ', slope
	else:
		control_error_steer = -(kp_steer_l*theta_error + ki_steer_l * steer_integral + kd_steer_l * steer_deriv )
		#print 'theta_error: ',theta_error
		#print 'steer_integral', steer_integral
		#print 'steer_deriv', steer_deriv
		print 'A solid -> RETA: ', slope
		print 'RETA : X2 - X1', x2-x1
		if var_print:
			print 'RETA: ', slope
	if var_print:
		print 'X2 - X1', x2-x1
	
	angle = control_error_steer
	print 'theta_error: ',theta_error
	print 'dist_corr: ',dist_corr
	print 'error_steer: ',error_steer
	print angle
		
	if angle > MAX_STEER:								#limita o angulo maximo
		angle = MAX_STEER
	if angle < -MAX_STEER:
		angle = -MAX_STEER
	
	
	#print "angle:",angle
	#--------------------------
	if var_print:
		print "-----------END Vehicle CONTROL ----------"

	return angle









if __name__ == '__main__':
	rospy.init_node('talker', anonymous=True)
	rospy.Subscriber('car1/' +'X_Y', coords, vehicle_control)
	pub= rospy.Publisher('drive_parameters', drive_param, queue_size=1)
	rospy.spin()
	
	
	
	
	
	
	
	
	
