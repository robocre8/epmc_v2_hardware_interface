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
const uint8_t READ_TVEL = 0x06;
const uint8_t SET_PPR = 0x07;
const uint8_t GET_PPR = 0x08;
const uint8_t SET_KP = 0x09;
const uint8_t GET_KP = 0x0A;
const uint8_t SET_KI = 0x0B;
const uint8_t GET_KI = 0x0C;
const uint8_t SET_KD = 0x0D;
const uint8_t GET_KD = 0x0E;
const uint8_t SET_RDIR = 0x0F;
const uint8_t GET_RDIR = 0x10;
const uint8_t SET_CUT_FREQ = 0x11;
const uint8_t GET_CUT_FREQ = 0x12;
const uint8_t SET_MAX_VEL = 0x13;
const uint8_t GET_MAX_VEL = 0x14;
const uint8_t SET_PID_MODE = 0x15;
const uint8_t GET_PID_MODE = 0x16;
const uint8_t SET_CMD_TIMEOUT = 0x17;
const uint8_t GET_CMD_TIMEOUT = 0x18;
const uint8_t SET_I2C_ADDR = 0x19;
const uint8_t GET_I2C_ADDR = 0x1A;
const uint8_t RESET_PARAMS = 0x1B;
const uint8_t SET_USE_IMU = 0x1C;
const uint8_t GET_USE_IMU = 0x1D;
const uint8_t READ_ACC = 0x1E;
const uint8_t READ_ACC_RAW = 0x1F;
const uint8_t READ_ACC_OFF = 0x20;
const uint8_t READ_ACC_VAR = 0x21;
const uint8_t WRITE_ACC_OFF = 0x22;
const uint8_t WRITE_ACC_VAR = 0x23;
const uint8_t READ_GYRO = 0x24;
const uint8_t READ_GYRO_RAW = 0x25;
const uint8_t READ_GYRO_OFF = 0x26;
const uint8_t READ_GYRO_VAR = 0x27;
const uint8_t WRITE_GYRO_OFF = 0x28;
const uint8_t WRITE_GYRO_VAR = 0x29;
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

  std::tuple<float, float, float, float> readPos()
  {
    auto [pos0, pos1, pos2, pos3] = read_data_stream(READ_POS);
    return {pos0, pos1, pos2, pos3};
  }

  std::tuple<float, float, float, float> readVel()
  {
    auto [v0, v1, v2, v3] = read_data_stream(READ_VEL);
    return {v0, v1, v2, v3};
  }

  std::tuple<float, float, float, float> readUVel()
  {
    auto [v0, v1, v2, v3] = read_data_stream(READ_UVEL);
    return {v0, v1, v2, v3};
  }


  int writePWM(int pwm0, int pwm1, int pwm2, int pwm3)
  {
    float res = write_data_stream(WRITE_PWM, (float)pwm0, (float)pwm1, (float)pwm2, (float)pwm3);
    return (int)res;
  }

  int writeSpeed(float v0, float v1, float v2, float v3)
  {
    float res = write_data_stream(WRITE_VEL, v0, v1, v2, v3);
    return (int)res;
  }

  int setCmdTimeout(int timeout_ms)
  {
    float res = write_data(SET_CMD_TIMEOUT, 0, (float)timeout_ms);
    return (int)res;
  }

  int getCmdTimeout()
  {
    float timeout_ms = read_data(GET_CMD_TIMEOUT, 0);
    return (int)timeout_ms;
  }

  int setPidMode(int motor_no, int mode)
  {
    float res = write_data(SET_PID_MODE, (uint8_t)motor_no, (float)mode);
    return (int)res;
  }

  int getPidMode(int motor_no)
  {
    float mode = read_data(GET_PID_MODE, (uint8_t)motor_no);
    return (int)mode;
  }

  int getUseIMU()
  {
    float mode = read_data(GET_USE_IMU, 0);
    return (int)mode;
  }

  std::tuple<float, float, float> readAcc()
  {
    auto [ax, ay, az, dummy] = read_data_stream(READ_ACC);
    return {ax, ay, az};
  }

  std::tuple<float, float, float> readAccVariance()
  {
    auto [ax, ay, az, dummy] = read_data_stream(READ_ACC_VAR);
    return {ax, ay, az};
  }

  std::tuple<float, float, float> readGyro()
  {
    auto [gx, gy, gz, dummy] = read_data_stream(READ_GYRO);
    return {gx, gy, gz};
  }

  std::tuple<float, float, float> readGyroVariance()
  {
    auto [gx, gy, gz, dummy] = read_data_stream(READ_GYRO_VAR);
    return {gx, gy, gz};
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  uint8_t calcChecksum(const std::vector<uint8_t>& packet) {
      uint32_t sum = 0;
      for (auto b : packet) sum += b;
      return sum & 0xFF;
  }

  void send_packet(uint8_t cmd) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, 0}; // no payload
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  void send_packet_stream(uint8_t cmd, const std::vector<uint8_t>& payload) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
      packet.insert(packet.end(), payload.begin(), payload.end());
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  float read_packet() {
      std::vector<uint8_t> payload(4);
      serial_conn_.Read(payload, 4);
      float val;
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
      return val;
  }

  std::tuple<float, float, float, float> read_packet_stream() {
      std::vector<uint8_t> payload(16);
      serial_conn_.Read(payload, 16);

      float vals[4];
      std::memcpy(&vals[0], payload.data() + 0, 4);
      std::memcpy(&vals[1], payload.data() + 4, 4);
      std::memcpy(&vals[2], payload.data() + 8, 4);
      std::memcpy(&vals[3], payload.data() + 12, 4);

      return {vals[0], vals[1], vals[2], vals[3]};
  }

  // ------------------- High-Level Wrappers -------------------
  float write_data(uint8_t cmd, uint8_t pos, float val) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_stream(cmd, payload);
      return read_packet();
  }

  float read_data(uint8_t cmd, uint8_t pos) {
      float zero = 0.0f;
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &zero, sizeof(float));
      send_packet_stream(cmd, payload);
      return read_packet();
  }

  float write_data_stream(uint8_t cmd, float a, float b, float c, float d) {
      std::vector<uint8_t> payload(4 * sizeof(float));
      std::memcpy(&payload[0],  &a, 4);
      std::memcpy(&payload[4],  &b, 4);
      std::memcpy(&payload[8],  &c, 4);
      std::memcpy(&payload[12], &d, 4);
      send_packet_stream(cmd, payload);
      return read_packet();
  }

  std::tuple<float, float, float, float> read_data_stream(uint8_t cmd) {
      send_packet(cmd);
      return read_packet_stream();
  }

};

#endif