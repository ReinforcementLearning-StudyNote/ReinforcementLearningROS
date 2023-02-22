#!/usr/bin/python3
import rospy
from uav import UAV
from environment.envs.PIDControl.script.pid import PID
from utility.common_func import *
from uav_visualization import UAV_Visualization
import math


if __name__ == '__main__':
	rospy.init_node(name='uav_att_pid', anonymous=False)

	pos0 = [0, 5, 0]  # 给的都是惯性系 东北天
	angle0 = [deg2rad(0), deg2rad(0), deg2rad(0)]  # 给的都是惯性系 东北天

	quad = UAV(pos0=pos0, angle0=angle0)  # initialization of a quadrotor
	quad_vis = UAV_Visualization()
	rate = rospy.Rate(1 / quad.dt)

	pid_x = PID(kp=5, ki=0., kd=450)  # controller of x
	pid_y = PID(kp=5, ki=0., kd=450)  # controller of y
	pid_z = PID(kp=2., ki=0., kd=250)  # controller z
	pid_phi = PID(kp=6, ki=0., kd=45)  # controller of roll along X in world
	pid_theta = PID(kp=6, ki=0., kd=45)  # controller of pitch along Y in world
	pid_psi = PID(kp=4, ki=0., kd=55)  # controller of yaw along Y in world

	xbound = np.array([quad.xmin, quad.xmax])
	ybound = np.array([quad.ymin, quad.ymax])
	zbound = np.array([quad.zmin, quad.zmax])
	origin = np.array([quad.x, quad.y, quad.z])
	f = np.array([0, 0, 0, 0])

	inv_coe_m = np.linalg.inv(quad.power_allocation_mat)  # 动力分配矩阵的逆
	while not (rospy.is_shutdown() or quad.is_episode_Terminal()):
		'''1. 生成参考轨迹'''
		phase = 2 * math.pi / 10 * quad.time
		'''八字'''
		# # 初值 [0, 4, 0]
		# x_ref = 6 * math.sin(phase) * math.cos(phase) / (1 + math.sin(phase) ** 2)
		# y_ref = 4 * math.cos(phase)/ (1 + math.sin(phase) ** 2)
		# z_ref = 1 * math.sin(phase)
		# psi_ref = deg2rad(0) * math.sin(math.pi * quad.time)

		'''圆'''
		# 初值 [0, 5, 0]
		x_ref = 5 * math.sin(phase)
		y_ref = 5 * math.cos(phase)
		z_ref = 1 * math.sin(phase)
		psi_ref = deg2rad(0) * math.sin(phase)
		t_ref = quad.time

		'''2. 计算位置 PID 控制的输出，同时得到期望姿态角'''
		ex, ey, ez = x_ref - quad.x, y_ref - quad.y, z_ref - quad.z
		pid_x.set_e(ex)
		pid_y.set_e(ey)
		pid_z.set_e(ez)
		ux, uy, uz = pid_x.out(), pid_y.out(), pid_z.out()
		U1 = quad.m * math.sqrt(ux ** 2 + uy ** 2 + (uz + quad.g) ** 2)
		phi_ref = math.asin((ux * math.sin(psi_ref) - uy * math.cos(psi_ref)) * quad.m / U1)
		theta_ref = math.asin((ux * quad.m - U1 * math.sin(psi_ref) * math.sin(phi_ref)) / (U1 * math.cos(psi_ref) * math.cos(phi_ref)))

		'''4. 计算姿态 PID 的控制输出'''
		e_phi, e_theta, e_psi = phi_ref - quad.phi, theta_ref - quad.theta, psi_ref - quad.psi
		pid_phi.set_e(e_phi)
		pid_theta.set_e(e_theta)
		pid_psi.set_e(e_psi)
		U2, U3, U4 = pid_phi.out(), pid_theta.out(), pid_psi.out()

		'''5. 动力分配'''
		square_omega = np.dot(inv_coe_m, [U1, U2, U3, U4])
		f = quad.CT * square_omega
		# print('PID out: U1: %.3f, U2: %.3f,U3: %.3f, U4: %.3f' % (U1, U2, U3, U4))
		for i in range(4):
			f[i] = max(min(quad.fmax, f[i]), quad.fmin)
		# print('Force UAV: ', f)
		quad.rk44(action=f)

		'''publish'''
		quad_vis.render(uav_pos=[quad.x, quad.y, quad.z],
						uav_pos_ref=[x_ref, y_ref, z_ref],
						uav_att=[quad.phi, quad.theta, quad.psi],
						uav_att_ref=[phi_ref, theta_ref, psi_ref],
						d=quad.d)
		rate.sleep()
	rospy.signal_shutdown('simulation over')
