# Hardware

## To buy:
- [1] Turtlebot 2e kit: http://www.turtlebot.com/turtlebot2/
- [2] Pine64 (at least 2GB) board: https://www.pine64.org/?product=pine-a64-board-2gb
- [3] Wifi dongle: https://www.amazon.com/Panda-Wireless-PAU06-300Mbps-Adapter/dp/B00JDVRCI0/ref=sr_1_1?ie=UTF8&qid=1499289096&sr=8-1&keywords=pau06 
- [4] Need a 12V -> 5V converter: https://www.amazon.com/HitCar-Inverter-Converter-Recorder-Straight/dp/B00U2DGKOK/ref=sr_1_1?s=electronics&ie=UTF8&qid=1494508390&sr=1-1&keywords=hitcar+dc+12v+to+5v
- [5] Need plugs to hook converter to Kobuki base: https://www.amazon.com/Headers-Housings-MiniFit-Recept-piece/dp/B00DEDIQ6K/ref=sr_1_2?s=industrial&ie=UTF8&qid=1494508471&sr=1-2&keywords=minifit+jr+conn+kit+plug+and+recept
- [6] A joystick: https://www.amazon.com/gp/product/B0041RR0TW/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1
- [7] A USB hub; any will do as long as it has at least 2 ports and doesn’t block the other USB ports. This is the one from the [turtlebot2e repo](https://github.com/turtlebot/turtlebot2e/blob/master/Accessories.md): http://www.newegg.com/Product/Product.aspx?Item=9SIA2BP0T23948
- [8] A microSD card for the Pine64; this is known to work: https://www.amazon.com/gp/product/B013CP5HCK/ref=oh_aui_detailpage_o04_s01?ie=UTF8&psc=1
- [9] A way to plug the microSD card into your Linux machine; if you don’t have one, buy this: https://www.amazon.com/gp/product/B00KX4TORI/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1
- [10] A heatsink for the Pine64; the ones for the Raspberry Pi will do: https://www.amazon.com/gp/product/B01GE7Q060/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1
- It can also be helpful for debugging to have an HDMI->HDMI cable [11] so you can hook the Pine64 to a monitor: https://www.amazon.com/AmazonBasics-High-Speed-HDMI-Cable-Standard/dp/B014I8SSD0/ref=sr_1_3?ie=UTF8&qid=1495646629&sr=8-3&keywords=HDMI+cable
 
## Setup:

- Build the turtlebot 2e according to the instructions that came with it.
- Put the heatsink (10) on the processor on the Pine64 (2).
- Connect the Pine64 wifi/bluetooth adapter to the top of the Pine64 (it only completely fits one way, but it is not keyed so be careful).
- Make the power cable from the kobuki to the Pine64 by taking the 12V->5V converter (4) and crimping the plugs (5) onto it.  Make sure to get power and ground correct; there are notes on the Kobuki website on the pinout: http://kobuki.yujinrobot.com/wiki/online-user-guide/
- Plug the USB hub into one of the Pine64 USB ports; plug the astra camera into the other one.
- Plug the USB A->B cable from the kobuki into the USB hub.
- Plug the cable from the joystick into the USB hub.
- Place the Pine64 onto the second deck of the Turtlebot.  Note that at this time it will just slide around; we’ll eventually make some kind of 3D printed case that you can screw down.
We’ll finish up once in the software section.
 
## Software

### Operating System setup
To get the Pine64 up and running, you need to put a special version of Ubuntu 16.04 (Xenial) on the microSD card.
- Start by downloading the disk image for the Pine64 from here: https://www.stdin.xyz/downloads/people/longsleep/pine64-images/ubuntu/
- Next, put the microSD card (8) into your microSD->USB converter (9), and plug it into your Linux machine.
- Once you plug it in, the device should show up as /dev/sdX; look in dmesg to determine the value of X.  Make sure you get this correct, because if you get it wrong the following commands can wipe your hard drive!
- Once you know the dev device that the microSD card is on, we can put the image on it:
  ```bash
  sudo apt install xz-utils
  xzcat xenial-pine64-bspkernel-20161218-1.img.xz | sudo dd of=/dev/sdX bs=1M oflag=sync
  ```
  Once that is complete, there is a bootable image on the microSD card.

- Before booting the board we need to tweak some network settings for future use. Unplug and replug your sdcard in your computer. And edit /etc/network/interfaces/eth0 and replace: `auto eth0 dhcp` with `allow-hotplug eth0 dhcp`
  We are now ready to boot up the Pine64 the first time.
- Insert the microSD card into the Pine64, plug an ethernet cable to give the pine internet access for the first configurations step, then plug the power cable we made during the hardware setup step into the microUSB port.  After a few seconds, the red light should come on on the Pine64, and if all goes well it should show up on your network.  If it doesn’t show up after a few minutes, the best thing to do is to plug it into a monitor with the HDMI cable and a keyboard and try to see what happened.
- Log into your pine (here we access it via ssh but you can also do this directly on the Pine64)
```bash
ssh ubuntu@<YOUR_PINE64_IP_ADDRESS>
```
It will ask you for a password, which is “ubuntu”.
- Update your board
	`sudo apt update && sudo apt upgrade`
- Configure the wireless interface:
  ```bash
  sudo apt install avahi-daemon linux-firmware nano network-manager 
  sudo nano /etc/network/interfaces.d/eth0
  ```
  Replace the first line `auto eth0` by `allow-hotplug eth0`
  ```bash
  sudo reboot
  sudo nmcli device wifi connect [SSID] password [PASSWORD] # (optionally specify interface but if only one wifi interface it’s not needed)
  ```
Your board should now show up on your network as “pine64.local”

One of the first things that should be done is to expand the size of the root partition.  To do this, run:
```bash
sudo /usr/local/sbin/resize_rootfs.sh
```

Which will take up all of the space on the microSD card for the root partition.
We should also remove the compressed RAM swap file.  While this is faster, it eats up half of our precious RAM (we’ll setup a file-based swap file in the next step).  To do this, run:
```bash
sudo apt-get purge zram-config
```

Next we’ll add a swap file.  Run these instructions:

```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo /swapfile none swap sw 0 0 | sudo tee -a /etc/fstab
```

Now you can follow the [instructions](https://github.com/ros2/turtlebot2_demo/tree/tb2_setup_instructions#installation) to install the turtlebot2 code
