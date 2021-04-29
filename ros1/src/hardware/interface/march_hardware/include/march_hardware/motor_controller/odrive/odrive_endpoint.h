#ifndef ODRIVE_ENDPOINT_HPP_
#define ODRIVE_ENDPOINT_HPP_

#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <endian.h>
#include <mutex>
#include <libusb-1.0/libusb.h>

// ODrive Device Info
#define ODRIVE_USB_VENDORID 0x1209
#define ODRIVE_USB_PRODUCTID 0x0D32

// ODrive USB Protocol
#define ODRIVE_TIMEOUT 1000
#define ODRIVE_MAX_BYTES_TO_RECEIVE 64
#define ODRIVE_MAX_RESULT_LENGTH 100
#define ODRIVE_DEFAULT_CRC_VALUE 0x9b40
#define ODRIVE_PROTOCOL_VERION 1

// ODrive Comm
#define ODRIVE_COMM_SUCCESS 0
#define ODRIVE_COMM_ERROR 1

// Endpoints (from target)
#define CDC_IN_EP 0x81     /* EP1 for data IN (target) */
#define CDC_OUT_EP 0x01    /* EP1 for data OUT (target) */
#define CDC_CMD_EP 0x82    /* EP2 for CDC commands */
#define ODRIVE_IN_EP 0x83  /* EP3 IN: ODrive device TX endpoint */
#define ODRIVE_OUT_EP 0x03 /* EP3 OUT: ODrive device RX endpoint */

// CDC Endpoints parameters
#define CDC_DATA_HS_MAX_PACKET_SIZE 64 /* Endpoint IN & OUT Packet size */
#define CDC_DATA_FS_MAX_PACKET_SIZE 64 /* Endpoint IN & OUT Packet size */
#define CDC_CMD_PACKET_SIZE 8          /* Control Endpoint Packet size */

#define USB_CDC_CONFIG_DESC_SIZ (67 + 39)
#define CDC_DATA_HS_IN_PACKET_SIZE CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE CDC_DATA_FS_MAX_PACKET_SIZE

#define CDC_SEND_ENCAPSULATED_COMMAND 0x00
#define CDC_GET_ENCAPSULATED_RESPONSE 0x01
#define CDC_SET_COMM_FEATURE 0x02
#define CDC_GET_COMM_FEATURE 0x03
#define CDC_CLEAR_COMM_FEATURE 0x04
#define CDC_SET_LINE_CODING 0x20
#define CDC_GET_LINE_CODING 0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22
#define CDC_SEND_BREAK 0x23

typedef std::vector<uint8_t> commBuffer;

namespace march
{
class OdriveEndpoint
{
public:
  /**
   * initialize USB library and local variables
   */
  OdriveEndpoint();

  /**
   * release USB library
   */
  ~OdriveEndpoint();

  /**
   * enumerate ODrive hardware
   * @param serialNumber odrive serial number
   * @return ODRIVE_COMM_SUCCESS on success else ODRIVE_COMM_ERROR
   */
  int open_connection(const std::string& serialNumber);

  /**
   * close ODrive device
   */
  void remove();

  /**
   *  Read value from ODrive
   *  @param id odrive ID
   *  @param value Data read
   *  @return ODRIVE_COMM_SUCCESS on success else ODRIVE_COMM_ERROR
   */
  template <typename T>
  int getData(int id, T& value);

  /**
   *  Write value to Odrive
   *  @param id odrive ID
   *  @param value Data to be written
   *  @return ODRIVE_COMM_SUCCESS on success
   */
  template <typename TT>
  int setData(int id, const TT& value);

  /**
   *  Request function to ODrive
   *  @param id odrive ID
   *  @return ODRIVE_COMM_SUCCESS on success
   */
  int execFunc(int id);

  /**
   * Request to USB endpoint
   * @param handle USB device handler
   * @param endpoint_id odrive ID
   * @param received_payload receive buffer
   * @param received_length receive length
   * @param payload data read
   * @param ack request acknowledge
   * @param length data length
   * @param read send read address
   * @param address read address
   * @return LIBUSB_SUCCESS on success
   */
  int endpointRequest(int endpoint_id, commBuffer& received_payload, int& received_length, const commBuffer& payload,
                      bool ack, int length, bool read = false, int address = 0);

  /**
   * Getter for the serial number
   * @return the serial number of the odrive connection
   */
  std::string getSerialNumber();

private:
  /**
   * Append short data to data buffer
   * @param buf data buffer
   * @param value data to append
   */
  static void appendShortToCommBuffer(commBuffer& buf, short value);

  /**
   * Append int data to data buffer
   * @param buf data buffer
   * @param value data to append
   */
  static void appendIntToCommBuffer(commBuffer& buf, int value);

  /**
   *  Decode odrive packet
   *  @param buf data buffer
   *  @param seq_no packet sequence number
   *  @param received_packet received buffer
   *  @return data buffer
   */
  static commBuffer decodeODrivePacket(commBuffer& buf, short& seq_no);

  /**
   * Read data buffer from Odrive hardware
   * @param seq_no next sequence number
   * @param endpoint_id USB endpoint ID
   * @param response_size maximum data length to be read
   * @param read append request address
   * @param address destination address
   * @param input data buffer to send
   * @return data buffer read
   */
  commBuffer createODrivePacket(short seq_no, int endpoint_id, short response_size, bool read, int address,
                                const commBuffer& input);

  libusb_context* lib_usb_context_ = nullptr;
  libusb_device_handle* odrive_handle_ = nullptr;

  bool attached_to_handle_;
  int crc_;

  int outbound_seq_no_ = 0;
  std::string odrive_serial_number;

  std::mutex ep_lock_;
};

}  // namespace march
#endif  // ODRIVE_ENDPOINT_HPP_
