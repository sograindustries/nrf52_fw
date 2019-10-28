#include <stdint.h>
#include <cstring>

#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "lib/ads/ads.h"

namespace argos {
  namespace ads {

    volatile bool spi_xfer_done;

    void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                           void *                    p_context)
    {
      spi_xfer_done = true;
    }
    
    ADS::ADS(const nrf_drv_spi_t& spi, const ads_pins_t& ads_pins):
      spi_(spi),
      pins_(ads_pins)
    {
      spi_xfer_done = false;
    }

    bool ADS::Init(){
  
      return true;
    }

    void ADS::WriteRegister(uint8_t address, uint8_t value) {
      memset(tx_buf_, 0, kBufferLength);
      memset(rx_buf_, 0, kBufferLength);
      spi_xfer_done = false;
      tx_buf_[0] = 0x40 + address;
      tx_buf_[1] = 1;
      tx_buf_[2] = value;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 3, rx_buf_, 3));

      while (!spi_xfer_done) {
        __WFE();
      }

      return;
    }

    uint8_t ADS::ReadRegister(uint8_t address) {
      memset(tx_buf_, 0, kBufferLength);
      memset(rx_buf_, 0, kBufferLength);
      spi_xfer_done = false;
      tx_buf_[0] = 0x20 + address;
      tx_buf_[1] = 1;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 3, rx_buf_, 3));

      while (!spi_xfer_done) {
        __WFE();
      }

      return rx_buf_[2];
    }

    void ADS::DebugWriteRegister(uint8_t address, uint8_t value) {
      uint8_t debug_value;
      debug_value = ReadRegister(address);
      NRF_LOG_INFO("Value %d: %x", address, debug_value);
      WriteRegister(address, value);
      debug_value = ReadRegister(address);
      NRF_LOG_INFO("Value %d: %x", address, debug_value);
      NRF_LOG_FLUSH();
    }

    void ADS::SendCommand(ADS1x9xCommand_t command){
      memset(tx_buf_, 0, kBufferLength);
      memset(rx_buf_, 0, kBufferLength);
      spi_xfer_done = false;
      tx_buf_[0] = command;
      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 1, rx_buf_, 1));

      while (!spi_xfer_done) {
        __WFE();
      }

      return;
    }

    int32_t ADS::GetData() {
      int32_t raw_value;
      memset(tx_buf_, 0, kBufferLength);
      memset(rx_buf_, 0, kBufferLength);
      spi_xfer_done = false;
      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi_, tx_buf_, 6, rx_buf_, 6));

      while (!spi_xfer_done) {
        __WFE();
      }

      raw_value = 0;
      if (rx_buf_[3] & 0x80) {
        raw_value = 0xff;
      }
      raw_value <<= 8;
      raw_value += rx_buf_[3];
      raw_value <<= 8;
      raw_value += rx_buf_[4];
      raw_value <<= 8;
      raw_value += rx_buf_[5];
      return raw_value;
    }
    
  }
}
