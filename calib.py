import numpy as np
import math
import transformations
from points import *

NOM = NOM.T
#NOM_A = NOM_A.T
MEAS = MEAS.T
#MEAS_A = MEAS_A.T
	
print "\nNOMINAL\n\n",NOM,"\n\nMEASURED\n\n",MEAS,"\n"
#print "\nNOMINAL_A\n\n",NOM_A,"\n\nMEASURED_A\n\n",MEAS_A,"\n"

#Rotation matrix may be pre- or post- multiplied (changing between a right-handed system and a left-handed system).
R = transformations.superimposition_matrix(MEAS,NOM,usesvd=True)
#R = transformations.inverse_matrix(R)
print "Rotation matrix calulated from points difference\n\n",R,"\n"
#rot=R.T
rob = transformations.euler_matrix(math.radians(OLDBASE[3]),math.radians(OLDBASE[4]),math.radians(OLDBASE[5]), axes='sxyz')
tob = transformations.translation_matrix(OLDBASE[0:3])
robn = np.dot(tob,rob)  #rotation matrix from old base
print "Rotation matrix from old base\n\n",robn,"\n"
robnew = np.dot(R,robn)
print "Rotation matrix RESULT\n\n",robnew,"\n"
scale, shear, angles, translate, perspective = transformations.decompose_matrix(robnew)

roll = math.degrees(angles[0])
pitch = math.degrees(angles[1])
yaw = math.degrees(angles[2])

print "OLD BASE(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n" %(OLDBASE[0],OLDBASE[1],OLDBASE[2],OLDBASE[3],OLDBASE[4],OLDBASE[5])
print "NEW BASE(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)\n" %(translate[0],translate[1],translate[2],roll,pitch,yaw)
print "----- COPY & PASTE -----\n\n[Base]"
print "BaseX = %.3f" % translate[0]
print "BaseY = %.3f" % translate[1]
print "BaseZ = %.3f" % translate[2]
print "BaseA = %.3f" % roll
print "BaseB = %.3f" % pitch
print "BaseC = %.3f\n\n------------------------" % yaw

raw_input()


