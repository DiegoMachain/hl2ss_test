import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from sympy import var, plot_implicit, Eq, symbols, Plane, solve
from sympy.plotting import plot3d

#Evaluation of simple QR
p1 = np.array([-0.2150, 0.1433, -0.5183])

p2 = np.array([-0.2745, 0.11123, -0.3165])


ref1 = np.array([[1,0,0], [0,1,0], [0,0,1]])
ref2 = np.array([[1,0,0], [0,1,0], [0,0,1]])

R1 = R.from_quat([0.6078625917434692,0.4554741382598877, -0.37643903493881226,0.5304149389266968])

R2 = R.from_quat([0.6978623270988464, 0.6696599721908569, -0.053607188165187836, 0.24833425879478455])

ref1 = R1.apply(ref1)
ref2 = R2.apply(ref2)

n1 = ref1[:, 1]

n2 = ref2[:, 1]

plane1 = Plane((p1[0], p1[1], p1[2]), (ref1[0,2], ref1[1,2], ref1[2,2]))
plane2 = Plane((p2[0], p2[1], p2[2]), (ref2[0,2], ref2[1,2], ref2[2,2]))

line = plane1.intersection(plane2)[0] 

print(line)
print(line.arbitrary_point())
print("")
print([plane1.equation(), plane2.equation()])

x = symbols('x')
y = symbols('y')
z = symbols('z')

plot3d(plane1.equation())




# pos_x: -0.2745012044906616
# pos_y: 0.11231803894042969
# pos_z: -0.316550612449646
# rot_x: 0.6078625917434692
# rot_y: 0.4554741382598877cha
# rot_z: -0.37643903493881226
# rot_w: 0.5304149389266968

# p2 =

# pos_x: -0.21524876356124878
# pos_y: 0.14421796798706055
# pos_z: -0.517582356929779
# rot_x: 0.6978623270988464
# rot_y: 0.6696599721908569
# rot_z: -0.053607188165187836
# rot_w: 0.24833425879478455



#Evaluation of manual adjustment
# pos_x: -0.24396783113479614
# pos_y: 0.09229958802461624
# pos_z: 0.46833550930023193
# rot_x: -0.2607804536819458
# rot_y: -0.5190863013267517
# rot_z: 0.6302829384803772
# rot_w: -0.5150595903396606