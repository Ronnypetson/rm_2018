import math

class GoToGoalPID:

	KpA = 1.5
	KiA = KpA / 10.0
	KdA = KpA * 5
	epsilonA = 0.01
	KpP = 0.7
	KdP = KpP * 5
	epsilonP = 0.1
	interationStop = 0

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
		return [u,-u]

	def goToPosition(self,roboX,roboY,roboA,goalX,goalY):
		y1 = roboY-goalY
		x1 = roboX-goalX
		roboA = self.normAngle(roboA)
		angGoal = self.normAngle(math.pi + math.atan2(y1, x1))
		deltaA = math.degrees(math.atan2(math.sin(angGoal-roboA), math.cos(angGoal-roboA)))
		sig = -1 if abs(deltaA) > 100 else 1
		self.errorP = sig * math.sqrt( ((goalX-roboX)**2) + ((goalY-roboY)**2) )
		self.diff_errorP = self.errorP - self.old_errorP
		u = (self.KpP * self.errorP) + (self.KdP * self.diff_errorP)
		self.old_errorP = self.errorP
		return [u,u]

	def goToGoal(self,roboX,roboY,roboA,goalX,goalY):
		if abs(math.degrees(self.errorA)) < self.epsilonA:
			if(abs(self.errorP) < self.epsilonP):
				return [0,0]
			else:
				#print("Position ", self.errorP)
				return self.goToPosition(roboX,roboY,roboA,goalX,goalY)
		else:
			#print("Angle ", math.degrees(self.errorA))
			return self.goToAngle(roboX,roboY,roboA,goalX,goalY)
