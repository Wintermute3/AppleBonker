#!/usr/bin/env python

# Import vpython library for 3d rendering and animation.

from visual import *

# Robot dimensions in mm

d3_e = 0.0
d3_f = 0.0
d3_re = 0.0
d3_rf = 0.0

# Trig constants

sqrt3 = math.sqrt(3)
pi = math.pi
sin120 = sqrt3/2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1.0/sqrt3

def d3_dimension(e, f, re, rf):
	global d3_e, d3_f, d3_re, d3_rf
	d3_e = e
	d3_f = f
	d3_re = re
	d3_rf = rf

# Global input/output variables for forward and inverse
# kinematics calculation routines.

d3_x0 = 0.0
d3_y0 = 0.0
d3_z0 = 0.0

d3_theta1 = 0.0
d3_theta2 = 0.0
d3_theta3 = 0.0

# Forward kinematics calculation.  Given the three servo angles (in
# degrees), return the coordinates of the end effector in the globals
# x0, y0 and z0.  Return 0 for success or -1 for error (non-existing
# point).

def d3_forward():
	global d3_x0, d3_y0, d3_z0, d3_theta1, d3_theta2, d3_theta3
	d3_x0 = d3_y0 = d3_z0 = 0.0
	t = (d3_f-d3_e)*tan30/2.0
	rad1 = deg2rad(d3_theta1)
	rad2 = deg2rad(d3_theta2)
	rad3 = deg2rad(d3_theta3)
	y1 = -(t + d3_rf*cos(rad1))
	z1 =     - d3_rf*sin(rad1)
	y2 =  (t + d3_rf*cos(rad2)) * sin30
	x2 = y2 * tan60
	z2 = - d3_rf * sin(rad2)
	y3 = (t + d3_rf*cos(rad3))*sin30
	x3 = -y3 * tan60
	z3 = -d3_rf * sin(rad3)
	dnm = (y2-y1)*x3-(y3-y1)*x2
	w1 = y1*y1 + z1*z1
	w2 = x2*x2 + y2*y2 + z2*z2
	w3 = x3*x3 + y3*y3 + z3*z3
	# x = (a1*z + b1)/dnm
	a1 = (z2-z1) * (y2-y2) - (z3-z1)*(y2-y2)
	b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
	# y = (a2*z + b2)/dnm
	a2 = -(z2-z1)*x3+(z3-z1)*x2
	b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0
	# a*z^2 + b*z + c = 0
	a = a1*a1 + a2*a2 + dnm*dnm
	b = 2.0*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
	c = (b2-y1*dnm)*(b2-y1*dnm)+b1*b1 + dnm*dnm*(z1*z1 - d3_re*d3_re)
	#discriminant
	d = b*b - 4.0*a*c
	if d < 0.0:
		return -1 # non-existent point
	d3_z0 = -0.5*(b + math.sqrt(d))/a
	d3_x0 = (a1*d3_z0 + b1) / dnm
	d3_y0 = (a2*d3_z0 + b2) / dnm
	return 0

# Inverse kinematics helper function.  Calculates angle theta (for
# yz-pane).

d3_theta = 0.0

def d3_angle(x0, y0, z0):
 	global d3_theta
	y1 = -0.5 * 0.57735 * d3_f # f/2 * tg 30
	y0 = y0 - 0.5 * 0.57735 * d3_e # shift center to edge
	# z = a + b*y
	a = (x0*x0 + y0*y0 + z0*z0 +d3_rf*d3_rf - d3_re*d3_re - y1*y1)/(2*z0)
	b = (y1-y0)/z0
	# discriminant
	d = -(a+b*y1)*(a+b*y1)+d3_rf*(b*b*d3_rf+d3_rf)
	if d < 0:
		return -1 # non-existing point
	yj = (y1 - a*b - sqrt(d))/(b*b + 1) # choosing outer point
	zj = a + b*yj
	if yj > y1:
		ax = 180.0
	else:
		ax = 0.0
	d3_theta = 180.0*math.atan(-zj/(y1 - yj))/pi + ax
	return 0

# Inverse kinematics calculation.  Given the three coordinates of
# the end effector, return the servo angles necessary to achieve
# that position in the globals x0, y0 and z0 (in degrees).  Return
# 0 for success or -1 for error (no solution).

def d3_inverse():
	global d3_x0, d3_y0, d3_z0, d3_theta1, d3_theta2, d3_theta3
	d3_theta1 = d3_theta2 = d3_theta3 = 0.0
	status = d3_angle(d3_x0, d3_y0, d3_z0)
	if status == 0:
		d3_theta1 = d3_theta
		status = d3_angle(d3_x0*cos120 + d3_y0*sin120, d3_y0*cos120-d3_x0*sin120, d3_z0)  # rotate coords to +120deg
	if status == 0:
		d3_theta2 = d3_theta
		status = d3_angle(d3_x0*cos120 - d3_y0*sin120, d3_y0*cos120+d3_x0*sin120, d3_z0)  # rotate coords to -120deg
	if status == 0:
		d3_theta3 = d3_theta
	return status

# Calculate and return the starting and ending positions for an
# arm.  Arm 1 is at azimuth 0, arm 2 at 120, and arm 3 at 240.
# Since arm 1 is in the yz plane at x=0, we calculate as if for
# arm 1 and the rotate to the desired target position.

def d3_arm(azimuth,theta):
	# start position for arm 1
	x = 0
	y = -math.tan(deg2rad(30))*d3_f/2
	z = 0
	# rotate to start position for target arm
	sx = (x*math.cos(deg2rad(azimuth))) - (y*math.sin(deg2rad(azimuth)))
	sy = (x*math.sin(deg2rad(azimuth))) + (y*math.cos(deg2rad(azimuth)))
	sz = 0
	# end offset from start position for arm 1
	x = 0
	y = -d3_rf*math.cos(deg2rad(theta))
	z = -d3_rf*math.sin(deg2rad(theta))
	# rotate to end offset from start position for target arm
	dx = (x*math.cos(deg2rad(azimuth))) - (y*math.sin(deg2rad(azimuth)))
	dy = (x*math.sin(deg2rad(azimuth))) + (y*math.cos(deg2rad(azimuth)))
	dz = z
	# translate to absolute ending position
	ex = sx + dx
	ey = sy + dy
	ez = sz + dz
	return (sx, sy, sz, dx, dy, dz, ex, ey, ez)

# Establish robot dimensions.

d3_dimension(115.0, 457.3, 232.0, 112.0)

scene = display(title='Delta3 Robot Kinematics', fullscreen=True)

# Render coordinate axis.

arrow(pos=(0,0,10), axis=(200,  0,  0), shaftwidth=6, color=color.white)
arrow(pos=(0,0,10), axis=(  0,200,  0), shaftwidth=6, color=color.white)
arrow(pos=(0,0,10), axis=(  0,  0,200), shaftwidth=6, color=color.white)

text(pos=(200, 20, 20), text='X', height=20, depth=5, color=color.red)
text(pos=( 20,200, 20), text='Y', height=20, depth=5, color=color.red)
text(pos=( 20, 20,200), text='Z', height=20, depth=5, color=color.red)

# Render base triangle.  This is an equilateral triangle in the xy
# plane with z=0, sides of d3_f, and one edge parallel to the x axis.

d3_base_shape = shapes.triangle(pos=(0,0), length=d3_f)
d3_fdz = 3
d3_base = extrusion(pos=[(0,0,0),(0,0,d3_fdz)], shape=d3_base_shape, color=color.yellow)

# Establish initial conditions.

d3_theta1 = 0
d3_theta2 = 0
d3_theta3 = 0

# Run the kinematics calculations forward and backward to test.

print('d3_forward(%.1f,%.1f,%.1f) -> %d' % (d3_theta1,d3_theta2,d3_theta3,d3_forward()))
print('[x0,y0,z0] = [%.1f,%.1f,%.1f]' % (d3_x0,d3_y0,d3_z0))

print('d3_inverse(%.1f,%.1f,%.1f) -> %d' % (d3_x0,d3_y0,d3_z0,d3_inverse()))
print('[theta1,theta2,theta3] = [%.1f,%.1f,%.1f]' % (d3_theta1,d3_theta2,d3_theta3))

print('d3_forward(%.1f,%.1f,%.1f) -> %d' % (d3_theta1,d3_theta2,d3_theta3,d3_forward()))
print('[x0,y0,z0] = [%.1f,%.1f,%.1f]' % (d3_x0,d3_y0,d3_z0))

# Render effector triangle.  This is an equilateral triangle in the xy
# plane with z=d3_z0, sides of d3_e, and one edge parallel to the x axis.

d3_eff_shape = shapes.triangle(pos=(0,0), length=d3_e)
d3_edz = 3
#d3_eff = extrusion(pos=[(d3_x0,d3_y0,d3_z0),(d3_x0,d3_y0,d3_z0-d3_edz)], shape=d3_eff_shape, color=color.red)

# Render servo arms.  Arms start from the midpoints of the base
# triangle sides.

a1 = d3_arm(  0, d3_theta1)
a2 = d3_arm(120, d3_theta2)
a3 = d3_arm(240, d3_theta3)

d3_a1 = cylinder(pos=(a1[0],a1[1],a1[2]), axis=(a1[3],a1[4],a1[5]), radius=6, color=color.green)
d3_a2 = cylinder(pos=(a2[0],a2[1],a2[2]), axis=(a2[3],a2[4],a2[5]), radius=6, color=color.green)
d3_a3 = cylinder(pos=(a3[0],a3[1],a3[2]), axis=(a3[3],a3[4],a3[5]), radius=6, color=color.green)

text(pos=(a1[0],a1[1],20), text='A1', height=20, depth=5, color=color.red)
text(pos=(a2[0],a2[1],20), text='A2', height=20, depth=5, color=color.red)
text(pos=(a3[0],a3[1],20), text='A3', height=20, depth=5, color=color.red)

for d3_x0 in range(-200,200,20):
	for d3_y0 in range(-200,200,20):
		for d3_z0 in range(-250,-50,20):
			rate(10)
			if d3_inverse() == 0:
				mark = ' '
				sphere(pos=(d3_x0,d3_y0,d3_z0), radius=3, color=color.green)
				a1 = d3_arm(  0, d3_theta1)
				a2 = d3_arm(120, d3_theta2)
				a3 = d3_arm(240, d3_theta3)
				d3_a1.axis=(a1[3],a1[4],a1[5])
				d3_a2.axis=(a2[3],a2[4],a2[5])
				d3_a3.axis=(a3[3],a3[4],a3[5])
			else:
				mark = '*'
				sphere(pos=(d3_x0,d3_y0,d3_z0), radius=2, color=color.red)
			print('%s %4.0f %4.0f %4.0f %4.0f %4.0f %4.0f' % (mark, d3_theta1, d3_theta2, d3_theta3, d3_x0, d3_y0, d3_z0))

print('Done!')