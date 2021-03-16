from environment_setup import calculateDistance
import math
import numpy as np
class long_lat_controller():
	def __init__(self):
		self.speed_increment   = 0.1  # target speed increase steps to avoid sudden jump in speed.
		self.L                 = 2    # Length of Vehicle
		self.l_d_min           = 2    # Setting a minimum lookahead distance   
		

		# Longitudinal controller gains
		self.Kp_longitudinal   = 0.25
		self.Ki_longitudinal   = 0.03
		self.Kd_longitudinal   = 0.1
		self.K_feedforward     = 0.1

		# Lateral controller gains
		self.Kpp               = 0.5  # Lateral controller gain

		# Initiating other variables
		self.longitudinal_error_previous = 0
		self.previous_t = 0
		self.current_t  = 0


	def find_lookahead_point(self):
		self.index = int(0)
		l_end = 0 # distance to end of path
		l_d   = 0 # look ahead distance


		# calculate distance to end of path
		l_end = calculateDistance(self.path.x[-1], self.path.y[-1], self.current_x, self.current_y)

		while True:

			# check current pos is not too close to end 
			if l_end < self.l_d_min:
				self.index = -1
				break
				
			# calc look ahead point
			l_d = calculateDistance(self.path.x[self.index], self.path.y[self.index], self.current_x, self.current_y)

			if l_d >= self.l_d_min:
				break

			# check reached look ahead distance
			if l_d >= self.l_d_min:
				break

			self.index += int(1)


	def step(self, path, current_x, current_y, current_yaw, current_speed):
		self.path          = path
		self.current_x     = current_x
		self.current_y     = current_y
		self.current_yaw   = current_yaw
		self.current_speed = current_speed
		self.set_speed     = self.path

		self.find_lookahead_point()

		self.set_speed     = self.path.s_d[self.index]

		self.longitudinal_control()

		self.lateral_control()

		self.longitudinal_error_previous = self.longitudinal_error
		self.previous_t                  = self.current_t
		
		return self.throttle, self.brake, self.steer

	def longitudinal_control(self):
		self.longitudinal_error            = self.set_speed - self.current_speed
		integral_longitudinal_error   = self.longitudinal_error + self.longitudinal_error_previous
		derivative_longitudinal_error = self.longitudinal_error - self.longitudinal_error_previous

		dt                            = self.current_t - self.previous_t


		if dt == 0:
			PID_output = self.Kp_longitudinal * self.longitudinal_error + \
						self.Ki_longitudinal * integral_longitudinal_error
		else:
			PID_output = self.Kp_longitudinal * self.longitudinal_error + \
						self.Ki_longitudinal * integral_longitudinal_error + \
						self.Kd_longitudinal * derivative_longitudinal_error / dt


		if PID_output == float('Inf'):
			PID_output = 0

		feedforward_control           = self.K_feedforward * self.set_speed

		combined_control              = feedforward_control + PID_output


		if combined_control >= 0:
			self.throttle = combined_control
			self.brake    = 0
		elif combined_control < 0:
			self.throttle = 0
			self.brake    = combined_control



	def lateral_control(self):
		if self.current_speed == 0:
			delta = 0
		else:
			alpha_hat = np.arctan2(self.path.y[self.index] - self.current_y, self.path.x[self.index] - self.current_x)
			alpha     = alpha_hat - self.current_yaw
			delta = np.arctan(2 * self.L * np.sin(alpha) / (self.Kpp * self.current_speed))

		self.steer = delta




