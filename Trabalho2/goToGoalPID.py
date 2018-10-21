import math

class GoToGoalPID:

	KpA = 0.5
	KiA = KpA / 10.0
	KdA = KpA * 5
	epsilonA = 0.001
	KpP = 2
	KiP = KpP / 10.0
	KdP = KpP * 5
	epsilonP = 0.01

	def __init__(self):
		self.errorA = self.epsilonA
		self.old_errorA = 0
		self.i_errorA = 0
		self.diff_errorA = 0
		self.errorP = self.epsilonP
		self.old_errorP = 0
		self.i_errorP = 0
		self.diff_errorP = 0

	def normAngle(self, theta):
		return theta % (2*math.pi)

	def goToAngle(self,roboX,roboY,roboA,goalX,goalY):
		roboA = self.normAngle(roboA)
		y1 = roboY-goalY
		x1 = roboX-goalX
		angGoal = self.normAngle(math.pi + math.atan2(y1, x1))
		self.errorA = math.atan2(math.sin(angGoal-roboA), math.cos(angGoal-roboA))
		self.i_errorA = self.i_errorA + self.errorA
		self.diff_errorA = self.errorA - self.old_errorA
		u = (self.KpA * self.errorA) + (self.KiA * self.i_errorA) + (self.KdA * self.diff_errorA)
		self.old_errorA = self.errorA;
		if u >= 0:
			return [u,0]
		else:
			return [0,-u]

	def goToPosition(self,roboX,roboY,roboA,goalX,goalY):
		y1 = roboY-goalY
		x1 = roboX-goalX
		roboA = self.normAngle(roboA)
		angGoal = abs(math.degrees(self.normAngle(math.pi + math.atan2(y1, x1))))
		sig = -1 if angGoal > 90 and angGoal < 270 else 1
		self.errorP = sig * math.sqrt( ((goalX-roboX)**2) + ((goalY-roboY)**2) )
		#self.i_errorP = self.i_errorP + self.errorP
		self.diff_errorP = self.errorP - self.old_errorP
		u = (self.KpP * self.errorP) + (self.KdP * self.diff_errorP) #+ (self.KiP * self.i_errorP)
		self.old_errorP = self.errorP
		return [u,u]


	def goToGoal(self,roboX,roboY,roboA,goalX,goalY):
		if abs(self.errorA) < self.epsilonA:
			#print("Position ", self.errorP)
			return self.goToPosition(roboX,roboY,roboA,goalX,goalY)
		else:
			#print("Angle ", self.errorA)
			return self.goToAngle(roboX,roboY,roboA,goalX,goalY)
