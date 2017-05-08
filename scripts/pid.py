#!/usr/bin/env python

class PID():
	
	def __init__(self, Kp, Ki, Kd, vMin, vMax):

		# ------ PARAMETRI ------
		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd
		self.vMin = vMin
		self.vMax = vMax
	
		# ------ VARIABILI ------
		self.err_old = 0.0
		self.integral = 0.0
		self.derivate = 0.0

	def calculate( self, measurement, ref, dt ):
	#""" ritorna una tensione V tra Vmin e Vmax  """
		err = ref - measurement

		# arresto
		if(ref == 0.0):
			self.reset()
			return 0.0 # tensione nulla

		self.update_integral(err, dt)

		self.update_derivate(err, dt)

		# calcolo azione di controllo
		v_input =  (self.Kp * err) + (self.Ki * self.integral) + (self.Kd * self.derivate)

		# saturatore
		v_input = self.saturation(v_input, err, dt)
		
		# aggiornamento dell'errore
		self.err_old = err

		return v_input

	def update_integral(self, err, dt):
		self.integral = self.integral + (err * dt)

	def update_derivate(self, err, dt):
		self.derivate = (err - self.err_old)/dt

	def saturation(self, v_input, err, dt):
		vSat = self.vMax-self.vMin
		if (v_input > vSat):
			v_input = vSat
			self.update_integral(-err, dt) # annullamento dell'ultimo accumulo dell'integrale
		
		elif (v_input < -vSat):
			v_input = -vSat
			self.update_integral(-err, dt) # annullamento dell'ultimo accumulo dell'integrale
			
		if (v_input > 0.0):
			return v_input + self.vMin
		elif (v_input < 0.0):
			return v_input - self.vMin

	def reset(self):
		self.integral = 0.0
		self.derivate = 0.0
