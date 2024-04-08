import math
import numpy as np
import bezier

def plant_trajectory(vx, tstride, overlay, swing_height, stance_height, standing_height):
	x_dataset = []
	z_dataset = []

	step_s = 0.0005 # s 

	T_stride_s = tstride/1000.0
	T_stance_s = T_stride_s * (0.5+overlay) # 0.200
	T_swing_s = T_stride_s * (0.5-overlay) # 0.200
	Vx_mps = vx/1000.0+0.001
	H_stance_m = stance_height/1000.0 # 0.005
	H_swing_m = swing_height/1000.0  # 0.040
	Z0 = -standing_height/1000.0

	x_begin = -Vx_mps * (0.5) * T_stance_s
	x_end   =  Vx_mps * (0.5) * T_stance_s

	xn = 8 # point count in x_points
	A = Vx_mps*T_swing_s/(xn-1)
	B = 2.00*A

	# X Bezier curve
	x_points = [ 
		x_begin,  	# C0 begin
		x_begin-A,  # C1 zero speed
		x_begin-B,  # C2 zero speed
		x_end/2,
		x_end/2,
		x_end+B,  	# C3 zero speed
		x_end+A,  	# C4 zero speed
		x_end,     	# C5
	]

	zn = 17 # point count in x_points
	A = 1.00/(zn-1)*H_stance_m*math.pi*T_swing_s/T_stance_s
	B = 2.00*A
	C = 2.00*B

	# Z Bezier curve
	z_points = [ 
			0.0,  	# C0
			A,  	# C1
			B,  	# C2
			H_swing_m*1.0,  # C3
			H_swing_m*1.0,  # C3
			H_swing_m*1.0,  # C3
			H_swing_m*1.0,  # C3
			H_swing_m*1.0,  # C3
			H_swing_m*1.0,  # C4
			H_swing_m*1.0,  # C5
			H_swing_m*1.2,  # C6
			H_swing_m*1.2,  # C7
			H_swing_m*1.0,  # C8
			C,  # C9
			B,  	# C10
			A,  	# C11
			0.0  	# C12
	]

	for t in np.arange(step_s ,T_stance_s, step_s):
		multiplier = t/T_stance_s
		x = -Vx_mps * (multiplier-0.5) * T_stance_s
		z = Z0-H_stance_m * math.cos(math.pi*(multiplier-0.5))

		x_dataset.append(x)
		z_dataset.append(z)


	for t in np.arange(0 ,(T_swing_s+step_s), step_s):
		multiplier = t/T_swing_s
		x = 0
		z = 0

		x = bezier.bezier1D(multiplier,x_points)
		z = bezier.bezier1D(multiplier,z_points)

		
		z = z + Z0

		x_dataset.append(x)
		z_dataset.append(z)


	data = np.column_stack((x_dataset,z_dataset))
	np.savetxt('quadruped_bezier_arc_coordinates.csv', data, delimiter=',',header = 'x, z', comments='')
	return data

# vx 1000
# tstride 400
# overlay 0
# swing_height 40
# stance_height 5
# standing_height 250

plant_trajectory(1000, 400, 0, 30, 5, 250)
