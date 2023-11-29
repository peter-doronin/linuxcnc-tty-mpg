all:
	gcc -I/usr/include/linuxcnc -I. -DULAPI -O4 tty_encoder.c -L/usr/lib -llinuxcnchal
