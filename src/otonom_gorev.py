#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def hedefe_git(bolge_adi, x, y, z_orient, w_orient):
    # Düğümü başlat (Eğer başlatılmadıysa)
    if rospy.get_name() == '/unnamed':
        rospy.init_node('otonom_kaptan')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo(f"Navigasyon sistemi bekleniyor... ({bolge_adi})")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # KONUM (Senin verdiğin veriler)
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0

    # YÖN (Senin verdiğin veriler)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = z_orient
    goal.target_pose.pose.orientation.w = w_orient

    rospy.loginfo(f"HEDEF: {bolge_adi} konumuna gidiliyor...")
    client.send_goal(goal)
    
    # Sonuç bekleniyor
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Sunucu hatası! Hedefe ulaşılamadı.")
    else:
        rospy.loginfo(f"VARILDI: {bolge_adi}. QR kodu okunuyor...")
        time.sleep(3) # 3 Saniye bekleme (QR okuma simülasyonu)

if __name__ == '__main__':
    try:
        rospy.init_node('otonom_kaptan')
        
        # --- SENİN KOORDİNAT LİSTEN ---
        rota = [
            # Bolge Adi, X, Y, Orientation_Z, Orientation_W
            ("Yatak Odasi", -5.5896, -0.2316, 0.7071, 0.7071),
            ("Salon",       -4.0610,  1.5813, -0.9999, 0.0126),
            ("Koridor",      0.9910,  0.3381,  0.7756, 0.6311),
            ("Mutfak",       3.2077,  1.4062,  0.9999, 0.0051)
        ]

        print("------------------------------------------------")
        print("BÜYÜK GÖREV BAŞLIYOR: 4 QR Kodu Gezilecek")
        print("------------------------------------------------")

        for bolge in rota:
            # Fonksiyonu çağır
            hedefe_git(bolge[0], bolge[1], bolge[2], bolge[3], bolge[4])

        print("------------------------------------------------")
        print("GÖREV TAMAMLANDI! Eve dönülüyor...")
        # İstersen buraya başlangıç noktasına dönüş kodu da eklenebilir
        print("------------------------------------------------")

    except rospy.ROSInterruptException:
        rospy.loginfo("Görev iptal edildi.")
