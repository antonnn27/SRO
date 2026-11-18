#!/usr/bin/env python3
"""waypoint_controller: P-controller menuju satu goal (closed loop dari pose).

Sub : /turtle1/pose (turtlesim/Pose), /mission_goal (MissionGoal)
Pub : /turtle1/cmd_vel (Twist), /mission_status (MissionStatus)
"""
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from waypoint_patrol.msg import MissionGoal, MissionStatus


class WaypointController:
    def __init__(self):
        self.kv = rospy.get_param('~kv', 1.5)
        self.kw = rospy.get_param('~kw', 4.0)
        self.tolerance = rospy.get_param('~tolerance', 0.08)
        self.rate = rospy.get_param('~rate', 20.0)
        self.pose = None
        self.goal = None

        self.pub_cmd = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pub_status = rospy.Publisher('/mission_status', MissionStatus, queue_size=10)

        rospy.Subscriber('/turtle1/pose', Pose, self.on_pose)
        rospy.Subscriber('/mission_goal', MissionGoal, self.on_goal)

        # Panggil loop() sebanyak rate kali per detik (20 kali).
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.loop)

    def on_pose(self, msg):
        self.pose = msg

    def on_goal(self, msg):
        self.goal = msg

    def loop(self, event):
        # Pose atau tujuan belum ada, atau misi sudah selesai: diam di tempat.
        if self.pose is None or self.goal is None or not self.goal.active:
            self.pub_cmd.publish(Twist())
            return

        # Selisih posisi (dx, dy) dan jarak d ke tujuan.
        dx = self.goal.x - self.pose.x
        dy = self.goal.y - self.pose.y
        d = math.hypot(dx, dy)
        sudut_target = math.atan2(dy, dx)
        e = sudut_target - self.pose.theta
        # Rapikan galat sudut ke rentang -pi sampai pi supaya belok lewat sisi terpendek.
        e = math.atan2(math.sin(e), math.cos(e))

        # Laporan untuk mission_manager: posisi sekarang dan sisa jarak.
        status = MissionStatus()
        status.goal_id = self.goal.goal_id
        status.x = self.pose.x
        status.y = self.pose.y
        status.distance_to_goal = d

        cmd = Twist()
        # Jarak lebih kecil dari toleransi: berhenti dan lapor sudah sampai.
        if d < self.tolerance:
            status.reached = True
        else:
            status.reached = False
            cmd.angular.z = self.kw * e
            # Maju hanya kalau arah sudah hampir benar. Kalau belum, putar dulu.
            if abs(e) < 0.3:
                cmd.linear.x = min(self.kv * d, 2.0)

        self.pub_cmd.publish(cmd)
        self.pub_status.publish(status)
        


if __name__ == '__main__':
    rospy.init_node('waypoint_controller')
    WaypointController()
    rospy.spin()
