Kobuki ftdi
===========

### Documentation ###

* [Official Web Page](http://kobuki.yujinrobot.com) - home page, sales, specifications and hardware howto.
* [Protocol, Usage and Api Documentation](http://yujinrobot.github.com/kobuki/doxygen/index.html) - in doxygen.

### Important Scripts ###

* create_udev_rules - creates /dev/kobuki link 
* get_serial_number
* flasher

### Trouble Shooting ###

##### What to check if kobuki does not bring up properly #####

* Does kobuki stream data?

> cat /dev/kobuki # check if any data stream happens

* Does kobuki appear as USB device?

> lsusb # See if there is "0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC"

> dmesg # See what happen when kobuki usb is plugged.

* Is there /dev/kobuki?

> rosrun kobuki_ftdi create_udev_rules

* Is Kobuki serial number correct?

> sudo ./get_serial_number # in the directory of kobuki_ftdi scripts

Check if it is different from below

<pre>
Device #0
  Manufacturer : Yujin Robot
  Product      : iClebo Kobuki
  Serial Number: kobuki_A601D86G
</pre>

If it is different,

> sudo ./flasher # in the directory of kobuki_ftdi scripts

Then check the serial again.






