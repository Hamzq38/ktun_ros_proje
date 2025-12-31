# Otonom Temizlik Robotu Projesi 

## 1. Kurulum (Installation)
Projeyi çalıştırmadan önce gerekli ROS paketlerini yüklemek için aşağıdaki komutları çalıştırın:

```bash
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3*
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-map-server
sudo apt-get install python3-zbar
```
## 2. Çalıştırma Komutları (Execution)
Adım 1: Projeyi Başlat
Navigasyon, AMCL ve Görev Yöneticisini (Task Manager) başlatır:
```
export ROSCONSOLE_CONFIG_FILE=~/ayar.config
roslaunch ktun_ros_proje proje_baslat.launch
```
Öncelikli görev atar:
```
rostopic pub /komut std_msgs/String "data: 'mutfak'" --once
```

## 3. Senaryo Açıklaması
-Başlangıç: Robot simülasyonda belirlenen başlangıç noktasında (x:-3.0, y:1.0) doğar.

-Hedefe Gidiş: config/mission.yaml dosyasından hedef odanın koordinatlarını okur ve otonom olarak (Move Base kullanarak) odaya gider.

-QR Tarama: Odaya vardığında kamera görüntüsünü işleyerek ortamdaki QR kodunu arar.

-Tarama: QR kodu tespit ettiğinde içeriğini okur, terminale basar.

-Görev Tamamlama: Odadaki belirlenen temizlik noktalarına gider, tamamlayınca bildirir.

## Demo Videosu [Proje Demo Videosunu İzlemek İçin Tıklayın](./video/robotik.mp4)
