snd-soc-wm8960-objs := wm8960.o
snd-soc-wm8960-soundcard-objs := wm8960-soundcard.o


obj-m += snd-soc-wm8960.o
obj-m += snd-soc-wm8960-soundcard.o
obj-m += JMO_DSI1024x600_Touch.o
obj-m += JMO_DSI1024x600_Screen.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp snd-soc-wm8960.ko /lib/modules/$(shell uname -r)/kernel/sound/soc/codecs/
	sudo cp snd-soc-wm8960-soundcard.ko /lib/modules/$(shell uname -r)/kernel/sound/soc/bcm/
	sudo cp JMO_DSI1024x600_Touch.ko /lib/modules/$(shell uname -r)/kernel/drivers/misc/
	sudo cp JMO_DSI1024x600_Screen.ko /lib/modules/$(shell uname -r)/kernel/drivers/misc/
	sudo depmod -a
