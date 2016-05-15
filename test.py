import numpy as np
import math
import transformations
from points import *

NOM = NOM.T
NOM_A = NOM_A.T
MEAS = MEAS.T
MEAS_A = MEAS_A.T
	
print "\nNOMINAL\n\n",NOM,"\n\nMEASURED\n\n",MEAS,"\n"
print "\nNOMINAL_A\n\n",NOM_A,"\n\nMEASURED_A\n\n",MEAS_A,"\n"

#Rotation matrix may be pre- or post- multiplied (changing between a right-handed system and a left-handed system).
R = transformations.superimposition_matrix(MEAS,NOM,usesvd=True)
scale, shear, angles, translate, perspective = transformations.decompose_matrix(R)
#R = transformations.inverse_matrix(R)
print "Rotation matrix\n\n",R,"\n"
#rot=R.T
p1 = transformations.euler_matrix(MEAS_A[0,0]/180*np.pi,MEAS_A[1,0]/180*np.pi,MEAS_A[2,0]/180*np.pi, axes='sxyz')
p2 = transformations.euler_matrix(MEAS_A[0,1]/180*np.pi,MEAS_A[1,1]/180*np.pi,MEAS_A[2,2]/180*np.pi, axes='sxyz')
p3 = transformations.euler_matrix(MEAS_A[0,2]/180*np.pi,MEAS_A[1,2]/180*np.pi,MEAS_A[2,2]/180*np.pi, axes='sxyz')
p4 = transformations.euler_matrix(MEAS_A[0,3]/180*np.pi,MEAS_A[1,3]/180*np.pi,MEAS_A[2,3]/180*np.pi, axes='sxyz')
t1 = transformations.translation_matrix(MEAS[0:,0])
t2 = transformations.translation_matrix(MEAS[0:,1])
t3 = transformations.translation_matrix(MEAS[0:,2])
t4 = transformations.translation_matrix(MEAS[0:,3])
#print "p1\n",p1,"\n","p2\n",p2,"\n","p3\n",p3,"\n","p4\n",p4,"\n"
#print "t1\n",t1,"\n","t2\n",t2,"\n","t3\n",t3,"\n","t4\n",t4,"\n"
p1n = np.dot(t1,p1)
p2n = np.dot(t2,p2)
p3n = np.dot(t3,p3)
p4n = np.dot(t4,p4)
print "p1n\n",p1n,"\n","p2n\n",p2n,"\n","p3n\n",p3n,"\n","p4n\n",p4n,"\n"
p1n = np.dot(R,p1n)
p2n = np.dot(R,p2n)
p3n = np.dot(R,p3n)
p4n = np.dot(R,p4n)
print "p1n\n",p1n,"\n","p2n\n",p2n,"\n","p3n\n",p3n,"\n","p4n\n",p4n,"\n"

roll, pitch, yaw = transformations.euler_from_matrix(p1n)
roll = math.degrees(roll)
pitch = math.degrees(pitch)
yaw = math.degrees(yaw)
print "roll (X):    %.3f" % roll
print "pitch(Y):    %.3f" % pitch
print "yaw  (Z):    %.3f \n" % yaw

roll, pitch, yaw = transformations.euler_from_matrix(p2n)
roll = math.degrees(roll)
pitch = math.degrees(pitch)
yaw = math.degrees(yaw)
print "roll (X):    %.3f" % roll
print "pitch(Y):    %.3f" % pitch
print "yaw  (Z):    %.3f \n" % yaw

roll, pitch, yaw = transformations.euler_from_matrix(p3n)
roll = math.degrees(roll)
pitch = math.degrees(pitch)
yaw = math.degrees(yaw)
print "roll (X):    %.3f" % roll
print "pitch(Y):    %.3f" % pitch
print "yaw  (Z):    %.3f \n" % yaw

roll, pitch, yaw = transformations.euler_from_matrix(p4n)
roll = math.degrees(roll)
pitch = math.degrees(pitch)
yaw = math.degrees(yaw)
print "roll (X):    %.3f" % roll
print "pitch(Y):    %.3f" % pitch
print "yaw  (Z):    %.3f \n" % yaw
MEAS = np.dot(R[0:3,0:3],MEAS)		#rotation
CORR = MEAS.T + translate
MEAS = CORR.T						#translation
"""
print "Rotation matrix inv\n\n",rot,"\n"
Roll, Pitch, Yaw = transformations.euler_from_matrix(R)
scale, shear, angles, translate, perspective = transformations.decompose_matrix(R)
Roll = math.degrees(Roll)
Pitch = math.degrees(Pitch)
Yaw = math.degrees(Yaw)

angles = np.dot((180/math.pi),angles)

print "scale ",scale,"\n"
print "shear ",shear,"\n"
print "angles ",angles,"\n"
print "translate ",translate,"\n"
print "perspective ",perspective,"\n"

MEAS = np.dot(R[0:3,0:3],MEAS)		#rotation
CORR = MEAS.T + translate
MEAS = CORR.T						#translation
print MEAS_A[1,0]
print MEAS_A[0,1]

for i in range(0,4):
	MEAS_A[0,i] = MEAS_A[0,i] + Roll
	MEAS_A[1,i] = MEAS_A[1,i] + Pitch
	MEAS_A[2,i] = MEAS_A[2,i] + Yaw
"""	
"""
print "CORRECTED ANGLES\n\n",MEAS_A,"\n"
MEAS_A = np.dot((math.pi/180),MEAS_A) # deg to rad
print "CORRECTED ANGLES RAD\n\n",MEAS_A,"\n"
MEAS_A = np.dot(rot[0:3,0:3],MEAS_A)		#rotation
print "CORRECTED ANGLES ROT\n\n",MEAS_A,"\n"
MEAS_A = np.dot((180/math.pi),MEAS_A) # rad to deg
print "CORRECTED ANGLES DEG\n\n",MEAS_A,"\n"
#CORR = MEAS_A.T + translate
#MEAS_A = CORR.T						#translation
#print "CORRECTED ANGLES TRANSLATE\n\n",MEAS_A,"\n"
"""
#print "	LC 		RC 		TC 		BC\n"
print "CORRECTED MEASURED POINTS\n\n",MEAS,"\n"
print "CORRECTED ANGLES\n\n",MEAS_A,"\n"
"""DMAT = MEAS.copy()
for i in range(0, 4):
	for j in range(0, 3):
		DMAT[j,i] = NOM[j,i] - MEAS[j,i]
print "DIFFERENCE\n\n",DMAT,"\n"""
BASE = np.array([R[0,3],R[1,3],R[2,3],Roll,Pitch,Yaw])
print "OLD BASE(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n" %(OLDBASE[0],OLDBASE[1],OLDBASE[2],OLDBASE[3],OLDBASE[4],OLDBASE[5])
print "DIF BASE(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n" %(BASE[0],BASE[1],BASE[2],BASE[3],BASE[4],BASE[5])
BASE = OLDBASE + BASE
print "NEW BASE(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n" %(BASE[0],BASE[1],BASE[2],BASE[3],BASE[4],BASE[5])
print "----- COPY & PASTE -----\n\n[Base]"
print "BaseX = %.3f" % BASE[0]
print "BaseY = %.3f" % BASE[1]
print "BaseZ = %.3f" % BASE[2]
print "BaseA = %.3f" % BASE[3]
print "BaseB = %.3f" % BASE[4]
print "BaseC = %.3f\n\n------------------------" % BASE[5]

raw_input()


