#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "spdlog/spdlog.h"

using dynamixel::GroupSyncRead;
using dynamixel::PacketHandler;
using dynamixel::PortHandler;

///////////////// for Protocol 2.0 Packet /////////////////
#define PKT_HEADER0 0
#define PKT_HEADER1 1
#define PKT_HEADER2 2
#define PKT_RESERVED 3
#define PKT_ID 4
#define PKT_LENGTH_L 5
#define PKT_LENGTH_H 6
#define PKT_INSTRUCTION 7
#define PKT_ERROR 8
#define PKT_PARAMETER0 8

#define TXPACKET_MAX_LEN (1 * 1024)
#define RXPACKET_MAX_LEN (1 * 1024)

unsigned short UpdateCRC(uint16_t crc_accum,
                         uint8_t* data_blk_ptr,
                         uint16_t data_blk_size) {
  uint16_t i;
  static const uint16_t crc_table[256] = {
      0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011, 0x8033,
      0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022, 0x8063, 0x0066,
      0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072, 0x0050, 0x8055, 0x805F,
      0x005A, 0x804B, 0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
      0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB,
      0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE,
      0x00B4, 0x80B1, 0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087,
      0x0082, 0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
      0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1, 0x01E0,
      0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6,
      0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
      0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176, 0x017C, 0x8179,
      0x0168, 0x816D, 0x8167, 0x0162, 0x8123, 0x0126, 0x012C, 0x8129, 0x0138,
      0x813D, 0x8137, 0x0132, 0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E,
      0x0104, 0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317,
      0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
      0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371, 0x8353,
      0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
      0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3, 0x03F6, 0x03FC,
      0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2, 0x83A3, 0x03A6, 0x03AC, 0x83A9,
      0x03B8, 0x83BD, 0x83B7, 0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B,
      0x038E, 0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E,
      0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7,
      0x02A2, 0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
      0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
      0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252, 0x0270, 0x8275,
      0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261, 0x0220, 0x8225, 0x822F,
      0x022A, 0x823B, 0x023E, 0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219,
      0x0208, 0x820D, 0x8207, 0x0202};

  for (uint16_t j = 0; j < data_blk_size; j++) {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

int RxPacketNew(PortHandler* port, uint8_t* rxpacket) {
  using clock = std::chrono::system_clock;
  using sec = std::chrono::duration<double>;

  auto t_start = clock::now();

  int result = COMM_TX_FAIL;

  uint16_t rx_length = 0;
  uint16_t wait_length =
      11;  // minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L
           // LENGTH_H INST ERROR CRC16_L CRC16_H)

  while (true) {
    auto t0 = clock::now();
    rx_length += port->readPort(&rxpacket[rx_length], wait_length - rx_length);
    auto t1 = clock::now();

    spdlog::info("readPort took {:.6f}s, rx_length = {}",
                 sec(t1 - t0).count(),
                 rx_length);

    if (rx_length >= wait_length) {
      uint16_t idx = 0;

      // find packet header
      for (idx = 0; idx < (rx_length - 3); idx++) {
        if ((rxpacket[idx] == 0xFF) && (rxpacket[idx + 1] == 0xFF) &&
            (rxpacket[idx + 2] == 0xFD) && (rxpacket[idx + 3] != 0xFD))
          break;
      }

      if (idx == 0)  // found at the beginning of the packet
      {
        t0 = clock::now();
        if (rxpacket[PKT_RESERVED] != 0x00 ||
            //           rxpacket[PKT_ID] > 0xFC || // FAST protocol responds
            //           with a broadcast ID
            DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) >
                RXPACKET_MAX_LEN ||
            rxpacket[PKT_INSTRUCTION] != 0x55) {
          // remove the first byte in the packet
          for (uint16_t s = 0; s < rx_length - 1; s++)
            rxpacket[s] = rxpacket[1 + s];
          // memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
          rx_length -= 1;
          spdlog::info("A: {:.6f}s", sec(clock::now() - t0).count());
          continue;
        }

        t0 = clock::now();
        // re-calculate the exact length of the rx packet
        if (wait_length !=
            DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) +
                PKT_LENGTH_H + 1) {
          wait_length =
              DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) +
              PKT_LENGTH_H + 1;
          spdlog::info("B: {:.6f}s", sec(clock::now() - t0).count());
          continue;
        }

        t0 = clock::now();
        if (rx_length < wait_length) {
          // check timeout
          if (port->isPacketTimeout() == true) {
            if (rx_length == 0) {
              result = COMM_RX_TIMEOUT;
            } else {
              result = COMM_RX_CORRUPT;
            }
            spdlog::info("C0: {:.6f}s", sec(clock::now() - t0).count());
            break;
          } else {
            spdlog::info("C1: {:.6f}s", sec(clock::now() - t0).count());
            continue;
          }
        }

        t0 = clock::now();
        // verify CRC16
        uint16_t crc =
            DXL_MAKEWORD(rxpacket[wait_length - 2], rxpacket[wait_length - 1]);
        if (UpdateCRC(0, rxpacket, wait_length - 2) == crc) {
          result = COMM_SUCCESS;
        } else {
          result = COMM_RX_CORRUPT;
        }
        spdlog::info("CRC16: {:.6f}s", sec(clock::now() - t0).count());
        break;
      } else {
        // remove unnecessary packets
        t0 = clock::now();
        for (uint16_t s = 0; s < rx_length - idx; s++)
          rxpacket[s] = rxpacket[idx + s];
        // memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx);
        rx_length -= idx;
        t1 = clock::now();
        spdlog::info("Removing took {:.6f}s", sec(t1 - t0).count());
      }
    } else {
      // check timeout
      if (port->isPacketTimeout() == true) {
        if (rx_length == 0) {
          result = COMM_RX_TIMEOUT;
        } else {
          result = COMM_RX_CORRUPT;
        }
        break;
      }
    }
    t0 = clock::now();
    usleep(10);
    spdlog::info("usleep: {:.6f}s", sec(clock::now() - t0).count());
  }
  port->is_using_ = false;

  spdlog::info("Before Return: {:.6f}s", sec(clock::now() - t_start).count());
  return result;
}

int ReadRxNew(PacketHandler* ph,
              PortHandler* port,
              uint8_t id,
              uint16_t length,
              uint8_t* data) {
  using clock = std::chrono::system_clock;
  using sec = std::chrono::duration<double>;

  auto t0 = clock::now();

  int result = COMM_TX_FAIL;
  std::vector<uint8_t> buf(1024);

  auto t1 = clock::now();

  int count = 0;

  do {
    ++count;
    result = RxPacketNew(port, buf.data());
  } while (result == COMM_SUCCESS && buf[4] != id);

  auto t2 = clock::now();

  spdlog::info(
      "ReaRxNew on id = {}, allocate: {:.6f}s, rxPacket: {:.6f}s"
      ", result = {}, buf[4] = {}, count = {}",
      id,
      sec(t1 - t0).count(),
      sec(t2 - t1).count(),
      result,
      buf[4],
      count);

  if (result == COMM_SUCCESS && buf[4] == id) {
    return COMM_SUCCESS;
  }

  return COMM_TX_FAIL;
}

int main(int argc, char** argv) {
  using clock = std::chrono::system_clock;
  using sec = std::chrono::duration<double>;

  bool result = false;

  // 1. Port Handler
  PortHandler* port_handler = PortHandler::getPortHandler("/dev/ttyDXL");
  if (!port_handler->openPort()) {
    spdlog::critical("Cannot open port!");
    std::abort();
  }

  if (!port_handler->setBaudRate(1'000'000)) {
    spdlog::critical("Cannot set baudrate!");
    std::abort();
  }

  // 2. Packet Handler
  PacketHandler* ph = PacketHandler::getPacketHandler(2.0f);

  // 3. Group Sync Read
  auto gsr = std::make_unique<GroupSyncRead>(port_handler, ph, 126, 137);

  spdlog::info("Call txRxPacket()");

  std::vector<uint8_t> buffer(137);

  gsr->clearParam();
  gsr->addParam(1);
  gsr->addParam(2);
  gsr->addParam(4);
  gsr->addParam(6);
  gsr->addParam(7);
  gsr->addParam(8);
  gsr->addParam(9);

  auto t0 = clock::now();

  int ret = gsr->txPacket();
  auto t1 = clock::now();
  spdlog::info("txPacket = {}, success = {}", ret, COMM_SUCCESS);

  // ret = ph->readRx(port_handler, 1, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 2, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 4, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 6, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 7, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 8, 137, buffer.data(), nullptr);
  // ret = ph->readRx(port_handler, 9, 137, buffer.data(), nullptr);

  ret = ReadRxNew(ph, port_handler, 1, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 2, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 4, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 6, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 7, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 8, 137, buffer.data());
  ret = ReadRxNew(ph, port_handler, 9, 137, buffer.data());

  // ret = gsr->rxPacket();
  auto t2 = clock::now();
  spdlog::info("rxPacket = {}, success = {}", ret, COMM_SUCCESS);

  spdlog::info("txPacket: {:.6f}s, rxPacket: {:.6f}s",
               sec(t1 - t0).count(),
               sec(t2 - t1).count());
  port_handler->closePort();

  return 0;
}
