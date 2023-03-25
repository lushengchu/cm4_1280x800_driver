1.
cm4_1280x800_driver function:
audio driver, 4G module bas on Quectel EC200/EC20, The touch and LCD we provide(1280x800 resolution)

2.
2023-02-21_Debian-Bullseys-with-the-Raspberry-Pi-Desktop-32-bit.img 
This version is based on Raspberry official and integrates cm4_1280x800_driver



3.
You can burn the official system, and install cm4_1280x800_driver manually:
3.1
sudo apt update
sudo apt-get -y install raspberrypi-kernel-headers raspberrypi-kernel 
sudo apt-get -y install  dkms i2c-tools libasound2-plugins
sudo apt-get -y install git
#reboot system
sudo reboot

3.2:
Enter cm4_1280x800_driver dir:
sudo ./install.sh
#Wait about 30 seconds, you can see finish tip
sudo reboot


