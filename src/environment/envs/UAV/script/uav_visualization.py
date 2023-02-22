import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
import numpy as np
import math


class UAV_Visualization:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/uav_pos', PoseStamped, queue_size=10)  # 发布无人机位姿
        self.marker_body_pub = rospy.Publisher('/rviz_body_marker', Marker, queue_size=10)  # 画机身
        self.marker_head_pub = rospy.Publisher('/rviz_head_marker', Marker, queue_size=10)  # 画机头
        self.marker_end_pub = rospy.Publisher('/rviz_end_marker', Marker, queue_size=10)  # 画端点
        self.traj_pub = rospy.Publisher('trajectory', Path, queue_size=10)  # 画轨迹
        self.traj_ref_pub = rospy.Publisher('trajectory_ref', Path, queue_size=10)  # 画轨迹

        self.uav_end = Marker()
        self.uav_body = Marker()
        self.uav_head = Marker()
        self.path = Path()
        self.path_ref = Path()

        self.path.header.frame_id = 'yyf_uav'
        self.path.header.stamp = rospy.Time.now()
        self.path_ref.header.frame_id = 'yyf_uav'
        self.path_ref.header.stamp = rospy.Time.now()

        self.uav_end.header.frame_id = 'yyf_uav'
        self.uav_end.type = Marker.SPHERE_LIST
        self.uav_end.colors = [ColorRGBA(r=1, g=0, b=0, a=1),
                               ColorRGBA(r=0, g=1, b=0, a=1),
                               ColorRGBA(r=0, g=0, b=1, a=1),
                               ColorRGBA(r=1, g=0.6, b=0, a=1)]
        self.uav_end.scale.x = 0.4
        self.uav_end.scale.y = 0.4
        self.uav_end.scale.z = 0.4
        self.uav_end.action = Marker.ADD

        self.uav_body.header.frame_id = 'yyf_uav'
        self.uav_body.type = Marker.LINE_LIST  # 3个杆
        self.uav_body.colors = [ColorRGBA(r=0, g=0, b=0, a=1) for _ in range(4)]
        self.uav_body.scale.x = 0.1  # linewidth
        self.uav_body.scale.y = 0.1  # linewidth
        self.uav_body.action = Marker.ADD

        self.uav_head.header.frame_id = 'yyf_uav'
        self.uav_head.type = Marker.ARROW
        self.uav_head.color = ColorRGBA(r=1, g=0., b=0., a=1)
        self.uav_head.scale.x = 0.1  # 箭柄
        self.uav_head.scale.y = 0.2  # 箭头
        self.uav_head.scale.z = 0.0
        self.uav_head.action = Marker.ADD

    @staticmethod
    def rotate_matrix(attitude: np.ndarray):
        [phi, theta, psi] = attitude
        _R_i_b1 = np.array([[math.cos(psi), math.sin(psi), 0],
                            [-math.sin(psi), math.cos(psi), 0],
                            [0, 0, 1]])  # 从惯性系到b1系，旋转偏航角psi
        _R_b1_b2 = np.array([[math.cos(theta), 0, -math.sin(theta)],
                             [0, 1, 0],
                             [math.sin(theta), 0, math.cos(theta)]])  # 从b1系到b2系，旋转俯仰角theta
        _R_b2_b = np.array([[1, 0, 0],
                            [0, math.cos(phi), math.sin(phi)],
                            [0, -math.sin(phi), math.cos(phi)]])  # 从b2系到b系，旋转滚转角phi
        _R_i_b = np.matmul(_R_b2_b, np.matmul(_R_b1_b2, _R_i_b1))  # 从惯性系到机体系的转换矩阵
        _R_b_i = _R_i_b.T  # 从机体系到惯性系的转换矩阵
        return _R_b_i

    def render(self, uav_pos, uav_pos_ref, uav_att, uav_att_ref, d):
        p = PoseStamped()
        p.header.frame_id = 'yyf_uav'
        p.header.stamp = rospy.Time.now()
        p.pose.position = Point(x=uav_pos[0], y=uav_pos[1], z=uav_pos[2])
        q = R.from_euler('zyx', [uav_att[2], uav_att[1], uav_att[0]]).as_quat()  # 无人机四元数
        p.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        p_ref = PoseStamped()
        p_ref.header.frame_id = 'yyf_uav'
        p_ref.header.stamp = rospy.Time.now()
        p_ref.pose.position = Point(x=uav_pos_ref[0], y=uav_pos_ref[1], z=uav_pos_ref[2])
        q_ref = R.from_euler('zyx', [uav_att_ref[2], uav_att_ref[1], uav_att_ref[0]]).as_quat()  # 参考无人机四元数
        p_ref.pose.orientation = Quaternion(x=q_ref[0], y=q_ref[1], z=q_ref[2], w=q_ref[3])

        self.path.poses.append(p)
        self.path_ref.poses.append(p_ref)

        self.pose_pub.publish(p)
        self.traj_pub.publish(self.path)
        self.traj_ref_pub.publish(self.path_ref)

        R_b_i = self.rotate_matrix(attitude=np.array(uav_att))
        center = np.array(uav_pos)
        d0 = d / math.sqrt(2)
        bar1 = np.dot(R_b_i, [d0, d0, 0]) + center
        bar2 = np.dot(R_b_i, [d0, -d0, 0]) + center
        bar3 = np.dot(R_b_i, [-d0, -d0, 0]) + center
        bar4 = np.dot(R_b_i, [-d0, +d0, 0]) + center
        head = np.dot(R_b_i, [2 * d0, 0, 0]) + center
        pt = [Point(x=bar1[0], y=bar1[1], z=bar1[2]),
              Point(x=bar2[0], y=bar2[1], z=bar2[2]),
              Point(x=bar3[0], y=bar3[1], z=bar3[2]),
              Point(x=bar4[0], y=bar4[1], z=bar4[2]),
              Point(x=center[0], y=center[1], z=center[2]),
              Point(x=head[0], y=head[1], z=head[2])]
        self.uav_body.points = [pt[0], pt[2], pt[1], pt[3]]
        self.uav_head.points = pt[4:6]
        self.uav_end.points = pt[0:4]
        self.marker_body_pub.publish(self.uav_body)
        self.marker_head_pub.publish(self.uav_head)
        self.marker_end_pub.publish(self.uav_end)
