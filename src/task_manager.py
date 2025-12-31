#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import yaml
import cv2
import time
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Pyzbar kontrolü
try:
    from pyzbar.pyzbar import decode
except ImportError:
    print("UYARI: pyzbar kütüphanesi yüklü değil! QR okuma çalışmayabilir.")
    def decode(image): return []

class GorevYoneticisi:
    def __init__(self):
        rospy.init_node('temizlik_gorev_yoneticisi')
        
        # --- AYARLAR ---
        self.yaml_path = os.path.expanduser("~/proje_ws/src/ktun_ros_proje/config/mission.yaml")
        
        # --- İSTEMCİLER VE ABONELİKLER ---
        self.bridge = CvBridge()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.kamera_callback)
        self.komut_sub = rospy.Subscriber("/komut", String, self.komut_callback)

        # --- YENİ EKLENTİ: 2D POSE ESTIMATE DİNLEYİCİSİ ---
        # Rviz'den konum atandığında bu topic tetiklenir
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.baslangic_konumu_callback)

        # --- DURUM DEĞİŞKENLERİ ---
        self.son_okunan_qr = ""
        self.qr_bulundu = False
        self.acil_gorev = None 
        self.gorev_iptal_bayragi = False
        self.konum_belirlendi = False # Robotun yeri belli mi?

        rospy.loginfo("Navigasyon sistemi bekleniyor...")
        if not self.client.wait_for_server(timeout=rospy.Duration(30.0)):
            rospy.logwarn("Navigasyon sunucusu bulunamadı!")
        else:
            rospy.loginfo("Sistem Hazır! Rviz'den Konumlandırma Bekleniyor...")

    def gorevleri_yukle(self):
        if not os.path.exists(self.yaml_path):
            rospy.logerr(f"Dosya bulunamadı: {self.yaml_path}")
            return None
        try:
            with open(self.yaml_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"YAML Hatası: {e}")
            return None

    def kamera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            decoded_objects = decode(cv_image)
            for obj in decoded_objects:
                self.son_okunan_qr = obj.data.decode("utf-8")
                self.qr_bulundu = True
        except:
            pass

    def komut_callback(self, msg):
        gelen_emir = msg.data.lower().strip()
        rospy.logwarn(f"!!! EMİR GELDİ: {gelen_emir.upper()} !!!")
        self.acil_gorev = gelen_emir
        self.gorev_iptal_bayragi = True 
        self.client.cancel_all_goals()
        # Eğer henüz başlamadıysa bile emir gelince başlat
        self.konum_belirlendi = True 

    def baslangic_konumu_callback(self, data):
        # Kullanıcı Rviz'de "2D Pose Estimate" yaptığı an burası çalışır
        rospy.loginfo("HARİKA! Konumlandırma alındı. Görev 3 saniye içinde başlıyor...")
        self.konum_belirlendi = True

    def robotu_dondur(self, hiz, sure):
        twist = Twist()
        twist.angular.z = hiz
        self.cmd_vel_pub.publish(twist)
        time.sleep(sure)
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def hedefe_git_guvenli(self, x, y, z_orient, w_orient):
        max_deneme = 2 
        for deneme in range(max_deneme):
            if self.gorev_iptal_bayragi: return False 

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = z_orient
            goal.target_pose.pose.orientation.w = w_orient

            self.client.send_goal(goal)
            wait = self.client.wait_for_result()
            
            if wait and self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return True
            else:
                rospy.logwarn(f"Hedefe gidilemedi! Deneme {deneme+1}/{max_deneme}")
                if deneme < max_deneme - 1:
                    rospy.loginfo("Tekrar deneniyor...")
                    time.sleep(1.0)
        return False

    def oda_operasyonu(self, oda_adi, config):
        if self.gorev_iptal_bayragi: return "IPTAL"
        if oda_adi not in config['gorev_listesi']:
            return "HATA"

        bilgi = config['gorev_listesi'][oda_adi]
        beklenen_qr = bilgi['qr_kodu']

        print(f"\n--- {oda_adi.upper()} GÖREVİ BAŞLATILIYOR ---")

        if not self.hedefe_git_guvenli(**bilgi['giris_noktasi']):
            return "HATA (Giriş)"

        if not self.hedefe_git_guvenli(**bilgi['qr_kontrol_noktasi']):
            return "HATA (QR Konumu)"

        rospy.loginfo("QR taranıyor...")
        self.qr_bulundu = False
        self.son_okunan_qr = ""
        qr_dogrulandi = False
        
        for deneme in range(3):
            if self.gorev_iptal_bayragi: return "IPTAL"
            start_t = time.time()
            while (time.time() - start_t) < 3.0:
                if self.qr_bulundu and self.son_okunan_qr == beklenen_qr:
                    qr_dogrulandi = True
                    break
                time.sleep(0.1)
            
            if qr_dogrulandi:
                rospy.loginfo(f"QR DOĞRULANDI: {self.son_okunan_qr}")
                break
            else:
                if deneme == 0:
                    rospy.logwarn("QR görülmedi, sağa hafif dönülüyor...")
                    self.robotu_dondur(-0.5, 1.0)
                elif deneme == 1:
                    rospy.logwarn("QR görülmedi, sola hafif dönülüyor...")
                    self.robotu_dondur(1.0, 1.0)

        if not qr_dogrulandi:
            rospy.logerr("QR Bulunamadı veya Yanlış! Oda Atlanıyor.")
            return "ATLANDI (QR Fail)"

        rospy.loginfo("Temizlik başlıyor...")
        self.client.cancel_all_goals()
        time.sleep(1.0)
        
        points = bilgi['temizlik_rotasi']
        tamamlandi = True
        for p in points:
            if self.gorev_iptal_bayragi: return "IPTAL"
            if not self.hedefe_git_guvenli(p['x'], p['y'], 0.0, 1.0):
                tamamlandi = False
        
        return "BAŞARILI" if tamamlandi else "BAŞARISIZ"

    def ana_dongu(self):
        config = self.gorevleri_yukle()
        if not config: return

        # --- BURASI YENİ: Robot konum almadan başlamaz ---
        print("\n" + "="*50)
        rospy.loginfo("BEKLENİYOR: Lütfen Rviz üzerinden '2D Pose Estimate' yapın...")
        print("="*50)
        
        while not self.konum_belirlendi and not rospy.is_shutdown():
            time.sleep(0.5)
        
        # Konum alındı, biraz bekle ki AMCL otursun
        time.sleep(3.0) 
        rospy.loginfo("Görev Başlıyor! Hedef: Yatak Odası")
        # ------------------------------------------------

        varsayilan_sira = ["yatak_odasi", "salon", "koridor", "mutfak"]
        rapor = {}

        while not rospy.is_shutdown():
            if self.acil_gorev:
                hedef_oda = self.acil_gorev
                self.acil_gorev = None
                self.gorev_iptal_bayragi = False
                
                rospy.loginfo(f"ACİL GÖREV İŞLENİYOR: {hedef_oda}")
                rapor[hedef_oda] = self.oda_operasyonu(hedef_oda, config)
                rospy.loginfo("Acil görev bitti. Yeni emir bekleniyor...")
                time.sleep(2)
            else:
                for oda in varsayilan_sira:
                    if self.acil_gorev: break
                    if oda in rapor and "BAŞARILI" in rapor[oda]: continue
                    rapor[oda] = self.oda_operasyonu(oda, config)
                
                if len(rapor) >= 4 and not self.acil_gorev:
                    print("\n" + "#"*40)
                    print("GENEL TEMİZLİK RAPORU")
                    for k, v in rapor.items(): print(f"{k.upper()}: {v}")
                    print("#"*40)
                    rospy.loginfo("Tüm görevler bitti. Beklemede...")
                    time.sleep(5) 

if __name__ == '__main__':
    try:
        yonetici = GorevYoneticisi()
        yonetici.ana_dongu()
    except rospy.ROSInterruptException:
        pass
