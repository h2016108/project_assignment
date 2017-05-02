# project_assignment

This is the kernel driver to interface gps module serially with raspberry pi. Steps to be performed to interface GPS module.

Download linux for raspberry pi from the link https://github.com/raspberrypi/linux. Store it in home directory
Run the following commands to build the kernel 

(i) cd linux 

(ii) KERNEL=kernel7 : selecting kernel for making object file

(iii) make bcm2709_defconfig & make menueconfig: configure .config file to add extra support to the existing kernel

(iv) Download all the codes from our repository. 

(v) Remove the existing driver from /linux/drivers/tty/serial/pl011.c because of resource conflict 

(vi) make -j4 zImage modules dtbs :- This step takes time.(approximate 7 hrs). -j4 indicates distribute the make to 4 cores of RPI.

(vii) sudo make modules_install : install object file to the kernel.

(viii) sudo cp arch/arm/boot/dts/.dtb /boot/ : copy boot drectory

(ix) sudo cp arch/arm/boot/dts/overlays/.dtb* /boot/overlays/ 

(x) sudo cp arch/arm/boot/dts/overlays/README /boot/overlays/ 

(xi) sudo cp arch/arm/boot/zImage /boot/$KERNEL.img : copy the new kernel image to the existing image.

(xii) reboot raspberry pi

The above Step build the kernel it creates the node named BITS_PILANI0 under /dev directory.

It creates class under /etc/class/tty/BITS_PILANI0

To run gps module follow these steps. 

(i) keep the GPS on open sky: for initial set up. 

(ii) sudo cat /dev/BITS_PILANI0 : it will produce value of gps in NMEA protocol format. The user will not understand this NMEA protocol. 

(iii) install GPSD by using the command "sudo apt-get install gpsd".

(iv) sudo nano /etc/default/gpsd.config. ->change the file, so that it points to the node '/dev/BITS_PILANI0' and save it

(v) run 'cgps -s' command to see the gps location.

for more information refer to the document
