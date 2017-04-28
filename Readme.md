This is the kernel driver to interface gps module serially with raspberry pi.
Steps to be performed to interface GPS module.
1) Download linux for raspberry pi from the link https://github.com/raspberrypi/linux. Store it in home directory
2) Run the following commands to build the kernel
	(i) cd linux
	(ii) KERNEL=kernel7
	(iii) make bcm2709_defconfig 
	(iv) Download all the codes from our repository.
	(v) Remove the existing driver from /linux/drivers/tty/serial/pl011.c because of resource conflict.
	(vi) make -j4 zImage modules dtbs :- This step takes time. 
	(vii) sudo make modules_install
	(viii) sudo cp arch/arm/boot/dts/*.dtb /boot/
	(ix) sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
	(x) sudo cp arch/arm/boot/dts/overlays/README /boot/overlays/
	(xi) sudo cp arch/arm/boot/zImage /boot/$KERNEL.img
	(xii) reboot raspberry pi
3) The above Step build the kernel it creates the node named BITS_PILANI0 under /dev directory.
4) To run gps module follow these steps.
	(i) keep the GPS on open sky
	(ii) sudo cat /dev/BITS_PILANI0 : it will produce value of gps in unformated pattern. The user will not understand this NMEA protocol.
	(iii) install GPSD by using the command "sudo apt-get install gpsd".
	(iv) sudo nano /etc/default/gpsd.config.
		->change the file, so that it points to the node '/dev/BITS_PILANI0' and save it
	(v) run 'cgps -s' command to see the gps location.
	