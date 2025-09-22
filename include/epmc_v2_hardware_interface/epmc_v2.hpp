#ifndef EPMC_V2_HPP
#define EPMC_V2_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>
#include <cstring>  // memcpy
#include <libserial/SerialPort.h>

// Serial Protocol Command IDs -------------
const uint8_t START_BYTE = 0xAA;
const uint8_t WRITE_VEL = 0x01;
const uint8_t WRITE_PWM = 0x02;
const uint8_t READ_POS = 0x03;
const uint8_t READ_VEL = 0x04;
const uint8_t READ_UVEL = 0x05;
const uint8_t SET_PID_MODE = 0x15;
const uint8_t GET_PID_MODE = 0x16;
const uint8_t SET_CMD_TIMEOUT = 0x17;
const uint8_t GET_CMD_TIMEOUT = 0x18;
const uint8_t READ_MOTOR_DATA = 0x2A;
//---------------------------------------------

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  case 460800:
    return LibSerial::BaudRate::BAUD_460800;
  case 921600:
    return LibSerial::BaudRate::BAUD_921600;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class EPMC_V2
{

public:
  EPMC_V2() = default;

  void connect(const std::string &serial_device, int32_t baud_rate = 921600, int32_t timeout_ms = 100)
  {
    try {
      timeout_ms_ = timeout_ms;
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
      serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    } catch (const LibSerial::OpenFailed&) {
        std::cerr << "Failed to open serial port!" << std::endl;
    }
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  void readPos(float &pos0, float& pos1, float &pos2, float &pos3)
  {
    read_data4(READ_POS, pos0, pos1, pos2, pos3);
  }

  void readVel(float &v0, float& v1, float &v2, float &v3)
  {
    read_data4(READ_VEL, v0, v1, v2, v3);
  }

  void readUVel(float &v0, float& v1, float &v2, float &v3)
  {
    read_data4(READ_UVEL, v0, v1, v2, v3);
  }

  int writePWM(int pwm0, int pwm1, int pwm2, int pwm3)
  {
    float res = write_data4(WRITE_PWM, (float)pwm0, (float)pwm1, (float)pwm2, (float)pwm3);
    return (int)res;
  }

  int writeSpeed(float v0, float v1, float v2, float v3)
  {
    float res = write_data4(WRITE_VEL, v0, v1, v2, v3);
    return (int)res;
  }

  int setCmdTimeout(int timeout_ms)
  {
    float res = write_data1(SET_CMD_TIMEOUT, 0, (float)timeout_ms);
    return (int)res;
  }

  int getCmdTimeout()
  {
    float timeout_ms = read_data1(GET_CMD_TIMEOUT, 0);
    return (int)timeout_ms;
  }

  int setPidMode(int motor_no, int mode)
  {
    float res = write_data1(SET_PID_MODE, (uint8_t)motor_no, (float)mode);
    return (int)res;
  }

  int getPidMode(int motor_no)
  {
    float mode = read_data1(GET_PID_MODE, (uint8_t)motor_no);
    return (int)mode;
  }

  void readMotorData(float &pos0, float& pos1, float &pos2, float &pos3, float &v0, float& v1, float &v2, float &v3)
  {
    read_data8(READ_MOTOR_DATA, pos0, pos1, pos2, pos3, v0, v1, v2, v3);
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  uint8_t calcChecksum(const std::vector<uint8_t>& packet) {
      uint32_t sum = 0;
      for (auto b : packet) sum += b;
      return sum & 0xFF;
  }

  void send_packet_without_payload(uint8_t cmd) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, 0}; // no payload
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  void send_packet_with_payload(uint8_t cmd, const std::vector<uint8_t>& payload) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
      packet.insert(packet.end(), payload.begin(), payload.end());
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  float read_packet1() {
      std::vector<uint8_t> payload(4);
      serial_conn_.Read(payload, 4);
      float val;
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
      return val;
  }

  void read_packet3(float &val0, float &val1, float &val2) {
      std::vector<uint8_t> payload(12);
      serial_conn_.Read(payload, 12);

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
  }

  void read_packet4(float &val0, float &val1, float &val2, float &val3) {
      std::vector<uint8_t> payload(16);
      serial_conn_.Read(payload, 16);

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
  }

  void read_packet6(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5) {
      std::vector<uint8_t> payload(24);
      serial_conn_.Read(payload, 24);

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
  }

  void read_packet8(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7) {
      std::vector<uint8_t> payload(32);
      serial_conn_.Read(payload, 32);

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
      std::memcpy(&val6, payload.data() + 24, sizeof(float));
      std::memcpy(&val7, payload.data() + 28, sizeof(float));
  }

  // ------------------- High-Level Wrappers -------------------
  float write_data1(uint8_t cmd, uint8_t pos, float val) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_with_payload(cmd, payload);
      return read_packet1();
  }

  float read_data1(uint8_t cmd, uint8_t pos) {
      float zero = 0.0f;
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &zero, sizeof(float));
      send_packet_with_payload(cmd, payload);
      return read_packet1();
  }

  float write_data4(uint8_t cmd, float a, float b, float c, float d) {
      std::vector<uint8_t> payload(4 * sizeof(float));
      std::memcpy(&payload[0],  &a, 4);
      std::memcpy(&payload[4],  &b, 4);
      std::memcpy(&payload[8],  &c, 4);
      std::memcpy(&payload[12], &d, 4);
      send_packet_with_payload(cmd, payload);
      return read_packet1();
  }

  void read_data3(uint8_t cmd, float &a, float &b, float &c) {
      send_packet_without_payload(cmd);
      return read_packet3(a, b, c);
  }

  void read_data4(uint8_t cmd, float &a, float &b, float &c, float &d) {
      send_packet_without_payload(cmd);
      return read_packet4(a, b, c, d);
  }

  void read_data6(uint8_t cmd, float &a, float &b, float &c, float &d, float &e, float &f) {
      send_packet_without_payload(cmd);
      return read_packet6(a, b, c, d, e, f);
  }

  void read_data8(uint8_t cmd, float &a, float &b, float &c, float &d, float &e, float &f, float &g, float &h) {
      send_packet_without_payload(cmd);
      return read_packet8(a, b, c, d, e, f, g, h);
  }

};

#endif