#ifndef ADI_SERIAL_PORT_H
#define ADI_SERIAL_PORT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <string>
#include <vector>

class SerialPort
{
	public:
		SerialPort();
		int openPort(const std::string &device);
		void closePort();
		bool isOpened() const;
		bool flushPort();
		bool write_bytes(const std::vector<uint8_t> &, const double timeout = 1.0);
		bool read_bytes(std::vector<uint8_t> &, const double timeout = 1.0);
	private:
		int fd = -1;
};

#endif
