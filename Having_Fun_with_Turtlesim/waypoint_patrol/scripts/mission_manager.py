#!/usr/bin/env python3
"""mission_manager: mengatur urutan waypoint.

Sub : /mission_status (MissionStatus)
Pub : /mission_goal (MissionGoal)
Srv : /start_mission (StartMission)
"""
import rospy
from waypoint_patrol.msg import MissionGoal, MissionStatus
from waypoint_patrol.srv import StartMission, StartMissionResponse


class MissionManager:
    def __init__(self):
        self.waypoints = rospy.get_param('~waypoints')
        self.index = 0
        self.goal_id = 0
        self.running = False

        # latch=True: tujuan terakhir disimpan, jadi controller yang menyala belakangan tetap menerimanya.
        self.pub_goal = rospy.Publisher('/mission_goal', MissionGoal, queue_size=10, latch=True)
        rospy.Subscriber('/mission_status', MissionStatus, self.on_status)
        rospy.Service('start_mission', StartMission, self.on_start)

    def on_start(self, req):
        # Service mulai misi: tolak kalau misi masih berjalan.
        if self.running:
            return StartMissionResponse(False, 'Misi sedang berjalan')
        self.index = 0
        self.running = True
        self.kirim_goal()
        return StartMissionResponse(True, 'Misi dimulai')

    def kirim_goal(self):
        x, y = self.waypoints[self.index]
        self.goal_id += 1
        self.pub_goal.publish(MissionGoal(goal_id=self.goal_id, x=x, y=y, active=True))

    def on_status(self, msg):
        if not self.running:
            return
        # Pindah titik hanya jika laporan sampai milik tujuan terakhir. Laporan lama diabaikan.
        if msg.reached and msg.goal_id == self.goal_id:
            rospy.loginfo('Waypoint %d tercapai', self.index + 1)
            self.index += 1
            if self.index < len(self.waypoints):
                self.kirim_goal()
            else:
                self.running = False
                self.goal_id += 1
                # Semua titik selesai: kirim tujuan nonaktif supaya kura-kura berhenti.
                self.pub_goal.publish(MissionGoal(goal_id=self.goal_id, x=0.0, y=0.0, active=False))
                rospy.loginfo('MISI SELESAI')


if __name__ == '__main__':
    rospy.init_node('mission_manager')
    manager = MissionManager()
    if rospy.get_param('~auto_start', True):
        # Hanya menunggu node lain siap, bukan penentu gerak. Setelah itu misi mulai otomatis.
        rospy.sleep(2.0)
        manager.on_start(None)
    rospy.spin()
