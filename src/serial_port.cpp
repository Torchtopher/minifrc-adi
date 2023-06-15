#include "adi_driver/serial_port.h"
#include <ostream>
#include <iostream>
/**
 * @brief Constructor
 */
SerialPort::SerialPort()
{
}

/**
 * @brief Open device
 * @param device Device file name (e.g. /dev/spidev0.0)
 * @retval 0 Success
 * @retval -1 Failure
 */
int SerialPort::openPort(const std::string &device)
{
  fd = open(device.c_str(), O_RDWR | O_NONBLOCK);

  uint32_t speed = 244000;
  uint32_t mode = 3;

  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); // speed = 0.244 MHz
  ioctl(fd, SPI_IOC_WR_MODE32, &mode); // mode = 3 (required by datasheet)

  return 0;
}

/**
 * @brief Close device
 */
void SerialPort::closePort()
{
  close(fd);
}

/**
 * @brief Query if port is open
 */
bool SerialPort::isOpened() const
{
  return fd != -1; // hopefully -1 is unused in real fds
}

/**
 * @brief Flush serial port
 */
bool SerialPort::flushPort()
{
  // do things...
  return true;
}

/**
 *  @brief write_bytes to the port
 */
bool SerialPort::write_bytes(const std::vector<uint8_t>& tx_bytes, const double timeout)
{
  uint8_t *buffer = new uint8_t[tx_bytes.size()];
  std::copy(tx_bytes.begin(), tx_bytes.end(), buffer);
  // Throw away first byte since the driver expects that to go
  // to the USB to SPI converter
  write(fd, &buffer[1], tx_bytes.size() - 1);
  //std::cout << "Wrote " << buffer << std::endl;
  delete buffer;
  return true;
}

/**
 *  @brief read_bytes from the port
 */
bool SerialPort::read_bytes(std::vector<uint8_t>& rx_bytes, const double timeout)
{
  // Add 0xFF first byte since the driver expects that to come
  // from the USB to SPI converter
  rx_bytes[0] = 0xFF;
	read(fd, &rx_bytes[1], rx_bytes.size()-1);
  return true;
}
