// C library headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <time.h>
#include <sys/ioctl.h>


// For user space HAL RTAPI
#ifdef RTAPI
#undef RTAPI
#endif

#ifndef ULAPI
#define ULAPI
#endif

#include "hal.h"

int comp_id;
int serial_port;

void quit(void)
{
	printf("TTY_MPG: comp exit\n");
	close(serial_port);
	if(comp_id > 0) hal_exit(comp_id);
	exit(0);
}

static void exit_handler(int sig) {
    printf("TTY_MPG: comp terminated\n");
    quit();
}

uint16_t calc_crc(uint8_t *packet, uint8_t size)
{
    uint16_t crc_polynome = 0xa001;
    uint16_t crc = 0xffff;
    for (uint8_t j = 0; j < size; j++)
    {
        crc ^= packet[j];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x1)
            {
                crc >>= 1;
                crc ^= crc_polynome;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

int main(int argc, char* argv[]) {
	
	if(argc <= 1) {printf("wrong arg\n"); return -1;};	

	serial_port = open(argv[1], O_RDWR);
	if(serial_port <= 0) {printf("Open port '%s' error\n", argv[1]); return -1;};
	ioctl(serial_port, TIOCEXCL); //lock the device
	
	struct termios tty;
	if(tcgetattr(serial_port, &tty) != 0) {
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
		return -1;
	}

	tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
	tty.c_cflag |= CS8; // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; // Disable echo
	tty.c_lflag &= ~ECHOE; // Disable erasure
	tty.c_lflag &= ~ECHONL; // Disable new-line echo
	tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed


	tty.c_cc[VTIME] = 1;
	tty.c_cc[VMIN] = 0;

	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
		printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
		return -1;
	}
	// END OF TTY SETUP
	
	
	signal(SIGINT, exit_handler);
    signal(SIGTERM, exit_handler);
    atexit(quit);
    
	//HAL section
	comp_id = hal_init("tty_mpg");
	if(comp_id < 0) return comp_id;
	
	struct {
		hal_float_t 	*pos;
		hal_s32_t 	*pos_s;
		hal_bit_t 	*x_ena;
		hal_bit_t 	*y_ena;
		hal_bit_t 	*z_ena;
		hal_bit_t 	*a_ena;
		hal_bit_t	*ready;
		hal_float_t *scale_in_0;
		hal_float_t *scale_in_1;
		hal_float_t *scale_in_2;
		hal_float_t *scale_out;
		hal_s32_t 	*pockets_losted;
		hal_float_t *update_freq;
		hal_s32_t 	*debug;
	} * tty_hal;
		
	tty_hal = hal_malloc(sizeof(*tty_hal));
    if (tty_hal == NULL) {
        fprintf(stderr, "%s: ERROR: unable to allocate HAL shared memory\n", "tty_mpg");
        return -1;};
        
    int r;
    r = hal_pin_float_newf(HAL_OUT, &(tty_hal->pos), comp_id, "%s.pos", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_s32_newf(HAL_OUT, &(tty_hal->pos_s), comp_id, "%s.pos_s", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_bit_newf(HAL_OUT, &(tty_hal->x_ena), comp_id, "%s.x_ena", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_bit_newf(HAL_OUT, &(tty_hal->y_ena), comp_id, "%s.y_ena", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_bit_newf(HAL_OUT, &(tty_hal->z_ena), comp_id, "%s.z_ena", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_bit_newf(HAL_OUT, &(tty_hal->a_ena), comp_id, "%s.a_ena", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_bit_newf(HAL_OUT, &(tty_hal->ready), comp_id, "%s.ready", "tty_mpg");
    if (r != 0) return -1;
        r = hal_pin_float_newf(HAL_IN, &(tty_hal->scale_in_0), comp_id, "%s.scale_in_0", "tty_mpg");
    if (r != 0) return -1;
        r = hal_pin_float_newf(HAL_IN, &(tty_hal->scale_in_1), comp_id, "%s.scale_in_1", "tty_mpg");
    if (r != 0) return -1;
        r = hal_pin_float_newf(HAL_IN, &(tty_hal->scale_in_2), comp_id, "%s.scale_in_2", "tty_mpg");
    if (r != 0) return -1;
        r = hal_pin_float_newf(HAL_OUT, &(tty_hal->scale_out), comp_id, "%s.scale_out", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_float_newf(HAL_OUT, &(tty_hal->update_freq), comp_id, "%s.update_freq", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_s32_newf(HAL_OUT, &(tty_hal->pockets_losted), comp_id, "%s.pockets_losted", "tty_mpg");
    if (r != 0) return -1;
    r = hal_pin_s32_newf(HAL_OUT, &(tty_hal->debug), comp_id, "%s.debug", "tty_mpg");
    if (r != 0) return -1;
    
	hal_ready(comp_id);
	
	*tty_hal->scale_in_0 = 1; 
	*tty_hal->scale_in_1 = 10;
	*tty_hal->scale_in_2 = 100;
	
	struct timespec start, stop;
	
	struct {
		uint16_t CNT;
		uint16_t gpio;
		uint16_t crc;
		} read_buf;
		
	struct gpio_t {
		bool x_ena : 1;
		bool y_ena : 1;
		bool z_ena : 1;
		bool a_ena : 1;
		bool scale_0 : 1;
		bool scale_1 : 1;
		bool scale_2 : 1;
		} * gpio_t;
	gpio_t = (struct gpio_t *)&read_buf.gpio;
		
	int error_counter;
	while(1)
	{
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
		tcflush(serial_port, TCIFLUSH);
		uint16_t tx;
		tx++;
		int retval = write(serial_port, &tx, 2);
		usleep(10000);
		int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop);
		uint16_t crc = calc_crc((uint8_t *)&read_buf, sizeof(read_buf) -2);
		*tty_hal->debug = read_buf.gpio;
		if((num_bytes != 6) || (crc != read_buf.crc)) {
			*tty_hal->pockets_losted += 1;
			error_counter++;
		} else 
		{
			error_counter = 0;
		}

		if(error_counter > 10) *tty_hal->ready = 0; else *tty_hal->ready = 1;
		if(*tty_hal->ready)
		{
			*tty_hal->pos_s = (int16_t)read_buf.CNT;
			*tty_hal->pos = (double)(uint16_t)read_buf.CNT;
			*tty_hal->x_ena = !gpio_t->x_ena;
			*tty_hal->y_ena = !gpio_t->y_ena;
			*tty_hal->z_ena = !gpio_t->z_ena;
			*tty_hal->a_ena = !gpio_t->a_ena;
			*tty_hal->scale_out = 0;
			if(gpio_t->scale_0 == 0) *tty_hal->scale_out = *tty_hal->scale_in_0;
			if(gpio_t->scale_1 == 0) *tty_hal->scale_out = *tty_hal->scale_in_1;
			if(gpio_t->scale_2 == 0) *tty_hal->scale_out = *tty_hal->scale_in_2;
		}
		*tty_hal->update_freq = 1 / ((stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec) / 1e6);	
	}	


	return 0;
};
