//This file is make by regenerating RadioLib.h for Spresense
//https://github.com/jgromes/RadioLib

#ifndef FSK_H
#define FSK_H

#include <Arduino.h>
#include <SPI.h>


#define FSK_DEFAULT_SPI SPI4 //拡張ボードのみ使用時はSPI4を使用,Spresense本体使用時はSPI5を使用
#define FSK_DEFAULT_SPI_FREQUENCY 8E6
#define FSK_DEFAULT_SS_PIN 22
#define FSK_DEFAULT_RESET_PIN 9
#define FSK_DEFAULT_DIO0_PIN 2
#define FSK_DEFAULT_DIO1_PIN 3


#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1


//cloned from PhysicalLayer.h
#if !defined(_RADIOLIB_PHYSICAL_LAYER_H)
#define _RADIOLIB_PHYSICAL_LAYER_H

#include "../../TypeDef.h"
#include "../../Module.h"

// common IRQ values - the IRQ flags in RadioLibIrqFlags_t arguments are offset by this value
enum RadioLibIrqType_t {
  RADIOLIB_IRQ_TX_DONE = 0x00,
  RADIOLIB_IRQ_RX_DONE = 0x01,
  RADIOLIB_IRQ_PREAMBLE_DETECTED = 0x02,
  RADIOLIB_IRQ_SYNC_WORD_VALID = 0x03,
  RADIOLIB_IRQ_HEADER_VALID = 0x04,
  RADIOLIB_IRQ_HEADER_ERR = 0x05,
  RADIOLIB_IRQ_CRC_ERR = 0x06,
  RADIOLIB_IRQ_CAD_DONE = 0x07,
  RADIOLIB_IRQ_CAD_DETECTED = 0x08,
  RADIOLIB_IRQ_TIMEOUT = 0x09,
  RADIOLIB_IRQ_NOT_SUPPORTED = 0x1F, // this must be the last value, intentionally set to 31
};

// some commonly used default values - defined here to ensure all modules have the same default behavior
#define RADIOLIB_IRQ_RX_DEFAULT_FLAGS       ((1UL << RADIOLIB_IRQ_RX_DONE) | (1UL << RADIOLIB_IRQ_TIMEOUT) | (1UL << RADIOLIB_IRQ_CRC_ERR) | (1UL << RADIOLIB_IRQ_HEADER_VALID) | (1UL << RADIOLIB_IRQ_HEADER_ERR))
#define RADIOLIB_IRQ_RX_DEFAULT_MASK        ((1UL << RADIOLIB_IRQ_RX_DONE))
#define RADIOLIB_IRQ_CAD_DEFAULT_FLAGS      ((1UL << RADIOLIB_IRQ_CAD_DETECTED) | (1UL << RADIOLIB_IRQ_CAD_DONE))
#define RADIOLIB_IRQ_CAD_DEFAULT_MASK       ((1UL << RADIOLIB_IRQ_CAD_DETECTED) | (1UL << RADIOLIB_IRQ_CAD_DONE))

/*!
  \struct LoRaRate_t
  \brief Data rate structure interpretation in case LoRa is used
*/
struct LoRaRate_t {
  /*! \brief LoRa spreading factor */
  uint8_t spreadingFactor;
  
  /*! \brief LoRa bandwidth in kHz */
  float bandwidth;
  
  /*! \brief LoRa coding rate */
  uint8_t codingRate;
};

/*!
  \struct FSKRate_t
  \brief Data rate structure interpretation in case FSK is used
*/
struct FSKRate_t {
  /*! \brief FSK bit rate in kbps */
  float bitRate;
  
  /*! \brief FSK frequency deviation in kHz */
  float freqDev;
};

/*!
  \struct LrFhssRate_t
  \brief Data rate structure interpretation in case LR-FHSS is used
*/
struct LrFhssRate_t {
  /*! \brief Bandwidth */
  uint8_t bw;

  /*! \brief Coding rate */
  uint8_t cr;

  /*! \brief Grid spacing */
  bool narrowGrid;
};

/*!
  \union DataRate_t
  \brief Common data rate structure
*/
union DataRate_t {
  /*! \brief Interpretation for LoRa modems */
  LoRaRate_t lora;

  /*! \brief Interpretation for FSK modems */
  FSKRate_t fsk;

  /*! \brief Interpretation for LR-FHSS modems */
  LrFhssRate_t lrFhss;
};

/*!
  \struct CADScanConfig_t
  \brief Channel scan configuration interpretation in case LoRa CAD is used
*/
struct CADScanConfig_t {
  /*! \brief Number of symbols to consider signal present */
  uint8_t symNum;
  
  /*! \brief Number of peak detection symbols */
  uint8_t detPeak;
  
  /*! \brief Number of minimum detection symbols */
  uint8_t detMin;
  
  /*! \brief Exit mode after signal detection is complete - module-specific value */
  uint8_t exitMode;
  
  /*! \brief Timeout in microseconds */
  RadioLibTime_t timeout;

  /*! \brief Optional IRQ flags to set, bits offset by the value of RADIOLIB_IRQ_ */
  RadioLibIrqFlags_t irqFlags;

  /*! \brief Optional IRQ mask to set, bits offset by the value of RADIOLIB_IRQ_ */
  RadioLibIrqFlags_t irqMask;
};

/*!
  \struct RSSIScanConfig_t
  \brief Channel scan configuration interpretation in case RSSI threshold is used
*/
struct RSSIScanConfig_t {
  /*! \brief RSSI limit in dBm */
  float limit;
};

/*!
  \union ChannelScanConfig_t
  \brief Common channel scan configuration structure
*/
union ChannelScanConfig_t {
  /*! \brief Interpretation for modems that use CAD (usually LoRa modems)*/
  CADScanConfig_t cad;

  /*! \brief Interpretation for modems that use RSSI threshold*/
  RSSIScanConfig_t rssi;
};

struct StandbyConfig_t {
  /*! \brief Module-specific standby mode configuration. */
  uint8_t mode;
};

struct ReceiveConfig_t {
  /*! \brief  Raw timeout value. Some modules use this argument to specify operation mode (single vs. continuous receive). */
  uint32_t timeout;
  
  /*! \brief Sets the IRQ flags. */
  RadioLibIrqFlags_t irqFlags;
  
  /*! \brief Sets the mask of IRQ flags that will trigger the radio interrupt pin. */
  RadioLibIrqFlags_t irqMask;
  
  /*! \brief Packet length, needed for some modules under special circumstances (e.g. LoRa implicit header mode). */
  size_t len;
};

struct TransmitConfig_t {
  /*! \brief Binary data that will be transmitted. */
  const uint8_t* data;

  /*! \brief Length of binary data to transmit (in bytes). */
  size_t len;

  /*! \brief Node address to transmit the packet to. Only used in FSK mode. */
  uint8_t addr;
};

struct SleepConfig_t {
  /*! \brief Module-specific sleep mode configuration. */
  uint8_t mode;
};

union RadioModeConfig_t {
  /*! \brief Interpretation for standby mode */
  StandbyConfig_t standby;

  /*! \brief Interpretation for Rx mode */
  ReceiveConfig_t receive;

  /*! \brief Interpretation for Tx mode */
  TransmitConfig_t transmit;

  /*! \brief Interpretation for scanning */
  ChannelScanConfig_t scan;

  /*! \brief Interpretation for sleep mode */
  SleepConfig_t sleep;
};

/*!
  \enum ModemType_t
  \brief Type of modem, used by setModem.
*/
enum ModemType_t {
  RADIOLIB_MODEM_FSK = 0,
  RADIOLIB_MODEM_LORA,
  RADIOLIB_MODEM_LRFHSS,
};

/*!
  \enum RadioModeType_t
  \brief Basic radio operating modes, used by stageMode.
*/
enum RadioModeType_t {
  RADIOLIB_RADIO_MODE_NONE = 0,
  RADIOLIB_RADIO_MODE_STANDBY,
  RADIOLIB_RADIO_MODE_RX,
  RADIOLIB_RADIO_MODE_TX,
  RADIOLIB_RADIO_MODE_SCAN,
  RADIOLIB_RADIO_MODE_SLEEP,
};

/*!
  \class PhysicalLayer

  \brief Provides common interface for protocols that run on %LoRa/FSK modules, such as RTTY or LoRaWAN.
  Also extracts some common module-independent methods. Using this interface class allows to use the protocols
  on various modules without much code duplicity. Because this class is used mainly as interface,
  all of its virtual members must be implemented in the module class.
*/
class PhysicalLayer {
  public:

    /*! \brief  Frequency step of the synthesizer in Hz. */
    float freqStep;

    /*! \brief  Maximum length of packet that can be received by the module. */
    size_t maxPacketLength;

    // constructor

    /*!
      \brief Default constructor.
    */
    PhysicalLayer();

    // basic methods

    #if defined(RADIOLIB_BUILD_ARDUINO)
    /*!
      \brief Arduino Flash String transmit method.
      \param str Pointer to Arduino Flash String that will be transmitted.
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    int16_t transmit(__FlashStringHelper* fstr, uint8_t addr = 0);

    /*!
      \brief Arduino String transmit method.
      \param str Address of Arduino string that will be transmitted.
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    int16_t transmit(String& str, uint8_t addr = 0);
    #endif

    /*!
      \brief C-string transmit method.
      \param str C-string that will be transmitted.
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    int16_t transmit(const char* str, uint8_t addr = 0);

    /*!
      \brief Binary transmit method. Must be implemented in module class.
      \param data Binary data that will be transmitted.
      \param len Length of binary data to transmit (in bytes).
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    virtual int16_t transmit(const uint8_t* data, size_t len, uint8_t addr = 0);

    #if defined(RADIOLIB_BUILD_ARDUINO)
    /*!
      \brief Arduino String receive method.
      \param str Address of Arduino String to save the received data.
      \param len Expected number of characters in the message. Leave as 0 if expecting a unknown size packet
      \returns \ref status_codes
    */
    int16_t receive(String& str, size_t len = 0);
    #endif

    /*!
      \brief Sets module to sleep.
      \returns \ref status_codes
    */
    virtual int16_t sleep();

    /*!
      \brief Sets module to standby.
      \returns \ref status_codes
    */
    virtual int16_t standby();

    /*!
      \brief Sets module to a specific standby mode.
      \returns \ref status_codes
    */
    virtual int16_t standby(uint8_t mode);

    /*!
      \brief Sets module to received mode using its default configuration.
      \returns \ref status_codes
    */
    virtual int16_t startReceive();

    /*!
      \brief Interrupt-driven receive method. A DIO pin will be activated when full packet is received. 
      Must be implemented in module class.
      \param timeout Raw timeout value. Some modules use this argument to specify operation mode
      (single vs. continuous receive).
      \param irqFlags Sets the IRQ flags.
      \param irqMask Sets the mask of IRQ flags that will trigger the radio interrupt pin.
      \param len Packet length, needed for some modules under special circumstances (e.g. LoRa implicit header mode).
      \returns \ref status_codes
    */
    virtual int16_t startReceive(uint32_t timeout, RadioLibIrqFlags_t irqFlags = RADIOLIB_IRQ_RX_DEFAULT_FLAGS, RadioLibIrqFlags_t irqMask = RADIOLIB_IRQ_RX_DEFAULT_MASK, size_t len = 0);

    /*!
      \brief Binary receive method. Must be implemented in module class.
      \param data Pointer to array to save the received binary data.
      \param len Packet length, needed for some modules under special circumstances (e.g. LoRa implicit header mode).
      \returns \ref status_codes
    */
    virtual int16_t receive(uint8_t* data, size_t len);

    #if defined(RADIOLIB_BUILD_ARDUINO)
    /*!
      \brief Interrupt-driven Arduino String transmit method. Unlike the standard transmit method, this one is non-blocking.
      Interrupt pin will be activated when transmission finishes.
      \param str Address of Arduino String that will be transmitted.
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    int16_t startTransmit(String& str, uint8_t addr = 0);
    #endif

    /*!
      \brief Interrupt-driven Arduino String transmit method. Unlike the standard transmit method, this one is non-blocking.
      Interrupt pin will be activated when transmission finishes.
      \param str C-string that will be transmitted.
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    int16_t startTransmit(const char* str, uint8_t addr = 0);

    /*!
      \brief Interrupt-driven binary transmit method.
      \param data Binary data that will be transmitted.
      \param len Length of binary data to transmit (in bytes).
      \param addr Node address to transmit the packet to. Only used in FSK mode.
      \returns \ref status_codes
    */
    virtual int16_t startTransmit(const uint8_t* data, size_t len, uint8_t addr = 0);

    /*!
      \brief Clean up after transmission is done.
      \returns \ref status_codes
    */
    virtual int16_t finishTransmit();

    #if defined(RADIOLIB_BUILD_ARDUINO)
    /*!
      \brief Reads data that was received after calling startReceive method.
      \param str Address of Arduino String to save the received data.
      \param len Expected number of characters in the message. When set to 0, the packet length will be retrieved 
      automatically. When more bytes than received are requested, only the number of bytes requested will be returned.
      \returns \ref status_codes
    */
    int16_t readData(String& str, size_t len = 0);
    #endif

    /*!
      \brief Reads data that was received after calling startReceive method.
      \param data Pointer to array to save the received binary data.
      \param len Number of bytes that will be read. When set to 0, the packet length will be retrieved automatically.
      When more bytes than received are requested, only the number of bytes requested will be returned.
      \returns \ref status_codes
    */
    virtual int16_t readData(uint8_t* data, size_t len);

    /*!
      \brief Enables direct transmission mode on pins DIO1 (clock) and DIO2 (data). Must be implemented in module class.
      While in direct mode, the module will not be able to transmit or receive packets. Can only be activated in FSK mode.
      \param frf 24-bit raw frequency value to start transmitting at. Required for quick frequency shifts in RTTY.
      \returns \ref status_codes
    */
    virtual int16_t transmitDirect(uint32_t frf = 0);

    /*!
      \brief Enables direct reception mode on pins DIO1 (clock) and DIO2 (data). Must be implemented in module class.
      While in direct mode, the module will not be able to transmit or receive packets. Can only be activated in FSK mode.
      \returns \ref status_codes
    */
    virtual int16_t receiveDirect();

    // configuration methods

    /*!
      \brief Sets carrier frequency. Must be implemented in module class.
      \param freq Carrier frequency to be set in MHz.
      \returns \ref status_codes
    */
    virtual int16_t setFrequency(float freq);

    /*!
      \brief Sets FSK bit rate. Only available in FSK mode. Must be implemented in module class.
      \param br Bit rate to be set (in kbps).
      \returns \ref status_codes
    */
    virtual int16_t setBitRate(float br);

    /*!
      \brief Sets FSK frequency deviation from carrier frequency. Only available in FSK mode.
      Must be implemented in module class.
      \param freqDev Frequency deviation to be set (in kHz).
      \returns \ref status_codes
    */
    virtual int16_t setFrequencyDeviation(float freqDev);

    /*!
      \brief Sets GFSK data shaping. Only available in FSK mode. Must be implemented in module class.
      \param sh Shaping to be set. See \ref config_shaping for possible values.
      \returns \ref status_codes
    */
    virtual int16_t setDataShaping(uint8_t sh);

    /*!
      \brief Sets FSK data encoding. Only available in FSK mode. Must be implemented in module class.
      \param encoding Encoding to be used. See \ref config_encoding for possible values.
      \returns \ref status_codes
    */
    virtual int16_t setEncoding(uint8_t encoding);

    /*!
      \brief Set IQ inversion. Must be implemented in module class if the module supports it.
      \param enable True to use inverted IQ, false for non-inverted.
      \returns \ref status_codes
    */
    virtual int16_t invertIQ(bool enable);

    /*!
      \brief Set output power. Must be implemented in module class if the module supports it.
      \param power Output power in dBm. The allowed range depends on the module used.
      \returns \ref status_codes
    */
    virtual int16_t setOutputPower(int8_t power);

    /*!
      \brief Check if output power is configurable. Must be implemented in module class if the module supports it.
      \param power Output power in dBm. The allowed range depends on the module used.
      \param clipped Clipped output power value to what is possible within the module's range.
      \returns \ref status_codes
    */
    virtual int16_t checkOutputPower(int8_t power, int8_t* clipped);

    /*!
      \brief Set sync word. Must be implemented in module class if the module supports it.
      \param sync Pointer to the sync word.
      \param len Sync word length in bytes. Maximum length depends on the module used.
      \returns \ref status_codes
    */
    virtual int16_t setSyncWord(uint8_t* sync, size_t len);

    /*!
      \brief Set preamble length. Must be implemented in module class if the module supports it.
      \param len Preamble length in bytes. Maximum length depends on the module used.
      \returns \ref status_codes
    */
    virtual int16_t setPreambleLength(size_t len);
    
    /*!
      \brief Set data. Must be implemented in module class if the module supports it.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    virtual int16_t setDataRate(DataRate_t dr);

    /*!
      \brief Check the data rate can be configured by this module. Must be implemented in module class if the module supports it.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    virtual int16_t checkDataRate(DataRate_t dr);

    /*!
      \brief Query modem for the packet length of received payload. Must be implemented in module class.
      \param update Update received packet length. Will return cached value when set to false.
      \returns Length of last received packet in bytes.
    */
    virtual size_t getPacketLength(bool update = true);

    /*!
      \brief Gets RSSI (Recorded Signal Strength Indicator) of the last received packet.
      \returns RSSI of the last received packet in dBm.
    */
    virtual float getRSSI();

    /*!
      \brief Gets SNR (Signal to Noise Ratio) of the last received packet. Only available for LoRa modem.
      \returns SNR of the last received packet in dB.
    */
    virtual float getSNR();

    /*!
      \brief Get expected time-on-air for a given size of payload
      \param len Payload length in bytes.
      \returns Expected time-on-air in microseconds.
    */
    virtual RadioLibTime_t getTimeOnAir(size_t len);

    /*!
      \brief Calculate the timeout value for this specific module / series 
      (in number of symbols or units of time).
      \param timeoutUs Timeout in microseconds to listen for.
      \returns Timeout value in a unit that is specific for the used module.
    */
    virtual RadioLibTime_t calculateRxTimeout(RadioLibTime_t timeoutUs);

    /*!
      \brief Convert from radio-agnostic IRQ flags to radio-specific flags.
      \param irq Radio-agnostic IRQ flags.
      \returns Flags for a specific radio module.
    */
    uint32_t getIrqMapped(RadioLibIrqFlags_t irq);

    /*!
      \brief Check whether a specific IRQ bit is set (e.g. RxTimeout, CadDone).
      \param irq IRQ type to check, one of RADIOLIB_IRQ_*.
      \returns 1 when requested IRQ is set, 0 when it is not or RADIOLIB_ERR_UNSUPPORTED if the IRQ is not supported.
    */
    int16_t checkIrq(RadioLibIrqType_t irq);

    /*!
      \brief Set interrupt on specific IRQ bit(s) (e.g. RxTimeout, CadDone).
      Keep in mind that not all radio modules support all RADIOLIB_IRQ_ flags!
      \param irq Flags to set, multiple bits may be enabled. IRQ to enable corresponds to the bit index (RadioLibIrq_t).
      For example, if bit 0 is enabled, the module will enable its RADIOLIB_IRQ_TX_DONE (if it is supported).
      \returns \ref status_codes
    */
    int16_t setIrq(RadioLibIrqFlags_t irq);

    /*!
      \brief Clear interrupt on a specific IRQ bit (e.g. RxTimeout, CadDone).
      Keep in mind that not all radio modules support all RADIOLIB_IRQ_ flags!
      \param irq Flags to set, multiple bits may be enabled. IRQ to enable corresponds to the bit index (RadioLibIrq_t).
      For example, if bit 0 is enabled, the module will enable its RADIOLIB_IRQ_TX_DONE (if it is supported).
      \returns \ref status_codes
    */
    int16_t clearIrq(RadioLibIrqFlags_t irq);

    /*!
      \brief Read currently active IRQ flags.
      Must be implemented in module class.
      \returns IRQ flags.
    */
    virtual uint32_t getIrqFlags();

    /*!
      \brief Set interrupt on DIO1 to be sent on a specific IRQ bit (e.g. RxTimeout, CadDone).
      Must be implemented in module class.
      \param irq Module-specific IRQ flags.
      \returns \ref status_codes
    */
    virtual int16_t setIrqFlags(uint32_t irq);

    /*!
      \brief Clear interrupt on a specific IRQ bit (e.g. RxTimeout, CadDone).
      Must be implemented in module class.
      \param irq Module-specific IRQ flags.
      \returns \ref status_codes
    */
    virtual int16_t clearIrqFlags(uint32_t irq);

    /*!
      \brief Interrupt-driven channel activity detection method. Interrupt will be activated
      when packet is detected. Must be implemented in module class.
      \returns \ref status_codes
    */
    virtual int16_t startChannelScan();

    /*!
      \brief Interrupt-driven channel activity detection method. interrupt will be activated
      when packet is detected. Must be implemented in module class.
      \param config Scan configuration structure. Interpretation depends on currently active modem.
      \returns \ref status_codes
    */
    virtual int16_t startChannelScan(const ChannelScanConfig_t &config);

    /*!
      \brief Read the channel scan result
      \returns \ref status_codes
    */
    virtual int16_t getChannelScanResult();

    /*!
      \brief Check whether the current communication channel is free or occupied. Performs CAD for LoRa modules,
      or RSSI measurement for FSK modules.
      \returns RADIOLIB_CHANNEL_FREE when channel is free,
      RADIOLIB_PREAMBLE_DETECTEDwhen occupied or other \ref status_codes.
    */
    virtual int16_t scanChannel();

    /*!
      \brief Check whether the current communication channel is free or occupied. Performs CAD for LoRa modules,
      or RSSI measurement for FSK modules.
      \param config Scan configuration structure. Interpretation depends on currently active modem.
      \returns RADIOLIB_CHANNEL_FREE when channel is free,
      RADIOLIB_PREAMBLE_DETECTEDwhen occupied or other \ref status_codes.
    */
    virtual int16_t scanChannel(const ChannelScanConfig_t &config);

    /*!
      \brief Get truly random number in range 0 - max.
      \param max The maximum value of the random number (non-inclusive).
      \returns Random number.
    */
    int32_t random(int32_t max);

    /*!
      \brief Get truly random number in range min - max.
      \param min The minimum value of the random number (inclusive).
      \param max The maximum value of the random number (non-inclusive).
      \returns Random number.
    */
    int32_t random(int32_t min, int32_t max);

    /*!
      \brief Get one truly random byte from RSSI noise. Must be implemented in module class.
      \returns TRNG byte.
    */
    virtual uint8_t randomByte();

    /*!
      \brief Configure module parameters for direct modes. Must be called prior to "ham" modes like RTTY or AX.25.
      Only available in FSK mode.
      \returns \ref status_codes
    */
    int16_t startDirect();

    #if !RADIOLIB_EXCLUDE_DIRECT_RECEIVE
    /*!
      \brief Set sync word to be used to determine start of packet in direct reception mode.
      \param syncWord Sync word bits.
      \param len Sync word length in bits. Set to zero to disable sync word matching.
      \returns \ref status_codes
    */
    int16_t setDirectSyncWord(uint32_t syncWord, uint8_t len);

    /*!
      \brief Set interrupt service routine function to call when data bit is received in direct mode.
      Must be implemented in module class.
      \param func Pointer to interrupt service routine.
    */
    virtual void setDirectAction(void (*func)(void));

    /*!
      \brief Function to read and process data bit in direct reception mode. Must be implemented in module class.
      \param pin Pin on which to read.
    */
    virtual void readBit(uint32_t pin);

    /*!
      \brief Get the number of direct mode bytes currently available in buffer.
      \returns Number of available bytes.
    */
    int16_t available();

    /*!
      \brief Forcefully drop synchronization.
    */
    void dropSync();

    /*!
      \brief Get data from direct mode buffer.
      \param drop Drop synchronization on read - next reading will require waiting for the sync word again.
      Defaults to true.
      \returns Byte from direct mode buffer.
    */
    uint8_t read(bool drop = true);
    #endif

    /*!
      \brief Configure DIO pin mapping to get a given signal on a DIO pin (if available).
      \param pin Pin number onto which a signal is to be placed.
      \param value The value that indicates which function to place on that pin. See chip datasheet for details.
      \returns \ref status_codes
    */
    virtual int16_t setDIOMapping(uint32_t pin, uint32_t value);

    /*!
      \brief Sets interrupt service routine to call when a packet is received.
      \param func ISR to call.
    */
    virtual void setPacketReceivedAction(void (*func)(void));

    /*!
      \brief Clears interrupt service routine to call when a packet is received.
    */
    virtual void clearPacketReceivedAction();

    /*!
      \brief Sets interrupt service routine to call when a packet is sent.
      \param func ISR to call.
    */
    virtual void setPacketSentAction(void (*func)(void));

    /*!
      \brief Clears interrupt service routine to call when a packet is sent.
    */
    virtual void clearPacketSentAction();
    
    /*!
      \brief Sets interrupt service routine to call when a channel scan is finished.
      \param func ISR to call.
    */
    virtual void setChannelScanAction(void (*func)(void));

    /*!
      \brief Clears interrupt service routine to call when a channel scan is finished.
    */
    virtual void clearChannelScanAction();

    /*!
      \brief Set modem for the radio to use. Will perform full reset and reconfigure the radio
      using its default parameters.
      \param modem Modem type to set. Not all modems are implemented by all radio modules!
      \returns \ref status_codes
    */
    virtual int16_t setModem(ModemType_t modem);

    /*!
      \brief Get modem currently in use by the radio.
      \param modem Pointer to a variable to save the retrieved configuration into.
      \returns \ref status_codes
    */
    virtual int16_t getModem(ModemType_t* modem);

    /*!
      \brief Stage mode of the radio to be launched later using launchMode.
      \param mode Radio mode to prepare.
      \param cfg Configuration of this mode (mode-dependent).
      \returns \ref status_codes
    */
    virtual int16_t stageMode(RadioModeType_t mode, RadioModeConfig_t* cfg);

    /*!
      \brief Launch previously staged mode.
      \returns \ref status_codes
    */
    virtual int16_t launchMode();

    #if RADIOLIB_INTERRUPT_TIMING

    /*!
      \brief Set function to be called to set up the timing interrupt.
      For details, see https://github.com/jgromes/RadioLib/wiki/Interrupt-Based-Timing
      \param func Setup function to be called, with one argument (pulse length in microseconds).
    */
    void setInterruptSetup(void (*func)(uint32_t));

    /*!
      \brief Set timing interrupt flag.
      For details, see https://github.com/jgromes/RadioLib/wiki/Interrupt-Based-Timing
    */
    void setTimerFlag();

    #endif

#if !RADIOLIB_GODMODE
  protected:
#endif
    uint32_t irqMap[10] = { 0 };
    RadioModeType_t stagedMode = RADIOLIB_RADIO_MODE_NONE;

#if !RADIOLIB_EXCLUDE_DIRECT_RECEIVE
    void updateDirectBuffer(uint8_t bit);
#endif

#if !RADIOLIB_GODMODE
  private:
#endif

    #if !RADIOLIB_EXCLUDE_DIRECT_RECEIVE
    uint8_t bufferBitPos = 0;
    uint8_t bufferWritePos = 0;
    uint8_t bufferReadPos = 0;
    uint8_t buffer[RADIOLIB_STATIC_ARRAY_SIZE] = { 0 };
    uint32_t syncBuffer = 0;
    uint32_t directSyncWord = 0;
    uint8_t directSyncWordLen = 0;
    uint32_t directSyncWordMask = 0;
    bool gotSync = false;
    #endif

    virtual Module* getMod() = 0;

    // allow specific classes access the private getMod method
    friend class AFSKClient;
    friend class RTTYClient;
    friend class MorseClient;
    friend class HellClient;
    friend class SSTVClient;
    friend class AX25Client;
    friend class FSK4Client;
    friend class PagerClient;
    friend class BellClient;
    friend class FT8Client;
    friend class LoRaWANNode;
    friend class M17Client;
};

#endif

//cloned from SX127x.h

#if !defined(_RADIOLIB_SX1278_H)
#define _RADIOLIB_SX1278_H

#include "../../TypeDef.h"

#if !RADIOLIB_EXCLUDE_SX127X

#include "../../Module.h"
#include "SX127x.h"

// SX1278 specific register map
#define RADIOLIB_SX1278_REG_MODEM_CONFIG_3                      0x26
#define RADIOLIB_SX1278_REG_PLL_HOP                             0x44
#define RADIOLIB_SX1278_REG_TCXO                                0x4B
#define RADIOLIB_SX1278_REG_PA_DAC                              0x4D
#define RADIOLIB_SX1278_REG_FORMER_TEMP                         0x5B
#define RADIOLIB_SX1278_REG_BIT_RATE_FRAC                       0x5D
#define RADIOLIB_SX1278_REG_AGC_REF                             0x61
#define RADIOLIB_SX1278_REG_AGC_THRESH_1                        0x62
#define RADIOLIB_SX1278_REG_AGC_THRESH_2                        0x63
#define RADIOLIB_SX1278_REG_AGC_THRESH_3                        0x64
#define RADIOLIB_SX1278_REG_PLL                                 0x70

// SX1278 LoRa modem settings
// RADIOLIB_SX1278_REG_OP_MODE                                                MSB   LSB   DESCRIPTION
#define RADIOLIB_SX1278_HIGH_FREQ                               0b00000000  //  3     3   access HF test registers
#define RADIOLIB_SX1278_LOW_FREQ                                0b00001000  //  3     3   access LF test registers

// RADIOLIB_SX1278_REG_FRF_MSB + REG_FRF_MID + REG_FRF_LSB
#define RADIOLIB_SX1278_FRF_MSB                                 0x6C        //  7     0   carrier frequency setting: f_RF = (F(XOSC) * FRF)/2^19
#define RADIOLIB_SX1278_FRF_MID                                 0x80        //  7     0       where F(XOSC) = 32 MHz
#define RADIOLIB_SX1278_FRF_LSB                                 0x00        //  7     0             FRF = 3 byte value of FRF registers

// RADIOLIB_SX1278_REG_PA_CONFIG
#define RADIOLIB_SX1278_MAX_POWER                               0b01110000  //  6     4   max power: P_max = 10.8 + 0.6*MAX_POWER [dBm]; P_max(MAX_POWER = 0b111) = 15 dBm
#define RADIOLIB_SX1278_LOW_POWER                               0b00100000  //  6     4

// RADIOLIB_SX1278_REG_LNA
#define RADIOLIB_SX1278_LNA_BOOST_LF_OFF                        0b00000000  //  4     3   default LNA current

// SX127X_REG_MODEM_CONFIG_1
#define RADIOLIB_SX1278_BW_7_80_KHZ                             0b00000000  //  7     4   bandwidth:  7.80 kHz
#define RADIOLIB_SX1278_BW_10_40_KHZ                            0b00010000  //  7     4               10.40 kHz
#define RADIOLIB_SX1278_BW_15_60_KHZ                            0b00100000  //  7     4               15.60 kHz
#define RADIOLIB_SX1278_BW_20_80_KHZ                            0b00110000  //  7     4               20.80 kHz
#define RADIOLIB_SX1278_BW_31_25_KHZ                            0b01000000  //  7     4               31.25 kHz
#define RADIOLIB_SX1278_BW_41_70_KHZ                            0b01010000  //  7     4               41.70 kHz
#define RADIOLIB_SX1278_BW_62_50_KHZ                            0b01100000  //  7     4               62.50 kHz
#define RADIOLIB_SX1278_BW_125_00_KHZ                           0b01110000  //  7     4               125.00 kHz
#define RADIOLIB_SX1278_BW_250_00_KHZ                           0b10000000  //  7     4               250.00 kHz
#define RADIOLIB_SX1278_BW_500_00_KHZ                           0b10010000  //  7     4               500.00 kHz
#define RADIOLIB_SX1278_CR_4_5                                  0b00000010  //  3     1   error coding rate:  4/5
#define RADIOLIB_SX1278_CR_4_6                                  0b00000100  //  3     1                       4/6
#define RADIOLIB_SX1278_CR_4_7                                  0b00000110  //  3     1                       4/7
#define RADIOLIB_SX1278_CR_4_8                                  0b00001000  //  3     1                       4/8
#define RADIOLIB_SX1278_HEADER_EXPL_MODE                        0b00000000  //  0     0   explicit header mode
#define RADIOLIB_SX1278_HEADER_IMPL_MODE                        0b00000001  //  0     0   implicit header mode

// SX127X_REG_MODEM_CONFIG_2
#define RADIOLIB_SX1278_RX_CRC_MODE_OFF                         0b00000000  //  2     2   CRC disabled
#define RADIOLIB_SX1278_RX_CRC_MODE_ON                          0b00000100  //  2     2   CRC enabled

// RADIOLIB_SX1278_REG_MODEM_CONFIG_3
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_OFF                   0b00000000  //  3     3   low data rate optimization disabled
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_ON                    0b00001000  //  3     3   low data rate optimization enabled
#define RADIOLIB_SX1278_AGC_AUTO_OFF                            0b00000000  //  2     2   LNA gain set by REG_LNA
#define RADIOLIB_SX1278_AGC_AUTO_ON                             0b00000100  //  2     2   LNA gain set by internal AGC loop

// SX127X_REG_VERSION
#define RADIOLIB_SX1278_CHIP_VERSION                            0x12  // this is the "official" version listed in datasheet
#define RADIOLIB_SX1278_CHIP_VERSION_ALT                        0x13  // appears sometimes 
#define RADIOLIB_SX1278_CHIP_VERSION_RFM9X                      0x11  // this value is used for the RFM9x

// SX1278 FSK modem settings
// SX127X_REG_PA_RAMP
#define RADIOLIB_SX1278_NO_SHAPING                              0b00000000  //  6     5   data shaping: no shaping (default)
#define RADIOLIB_SX1278_FSK_GAUSSIAN_1_0                        0b00100000  //  6     5                 FSK modulation Gaussian filter, BT = 1.0
#define RADIOLIB_SX1278_FSK_GAUSSIAN_0_5                        0b01000000  //  6     5                 FSK modulation Gaussian filter, BT = 0.5
#define RADIOLIB_SX1278_FSK_GAUSSIAN_0_3                        0b01100000  //  6     5                 FSK modulation Gaussian filter, BT = 0.3
#define RADIOLIB_SX1278_OOK_FILTER_BR                           0b00100000  //  6     5                 OOK modulation filter, f_cutoff = BR
#define RADIOLIB_SX1278_OOK_FILTER_2BR                          0b01000000  //  6     5                 OOK modulation filter, f_cutoff = 2*BR

// RADIOLIB_SX1278_REG_AGC_REF
#define RADIOLIB_SX1278_AGC_REFERENCE_LEVEL_LF                  0x19        //  5     0   floor reference for AGC thresholds: AgcRef = -174 + 10*log(2*RxBw) + 8 + AGC_REFERENCE_LEVEL [dBm]: below 525 MHz
#define RADIOLIB_SX1278_AGC_REFERENCE_LEVEL_HF                  0x1C        //  5     0                                                                                                         above 779 MHz

// RADIOLIB_SX1278_REG_AGC_THRESH_1
#define RADIOLIB_SX1278_AGC_STEP_1_LF                           0x0C        //  4     0   1st AGC threshold: below 525 MHz
#define RADIOLIB_SX1278_AGC_STEP_1_HF                           0x0E        //  4     0                      above 779 MHz

// RADIOLIB_SX1278_REG_AGC_THRESH_2
#define RADIOLIB_SX1278_AGC_STEP_2_LF                           0x40        //  7     4   2nd AGC threshold: below 525 MHz
#define RADIOLIB_SX1278_AGC_STEP_2_HF                           0x50        //  7     4                      above 779 MHz
#define RADIOLIB_SX1278_AGC_STEP_3                              0x0B        //  3     0   3rd AGC threshold

// RADIOLIB_SX1278_REG_AGC_THRESH_3
#define RADIOLIB_SX1278_AGC_STEP_4                              0xC0        //  7     4   4th AGC threshold
#define RADIOLIB_SX1278_AGC_STEP_5                              0x0C        //  4     0   5th AGC threshold

/*!
  \class SX1278
  \brief Derived class for %SX1278 modules. Also used as base class for SX1276, SX1277, SX1279, RFM95 and RFM96.
  All of these modules use the same basic hardware and only differ in parameter ranges (and names).
*/
class SX1278: public SX127x {
  public:

    // constructor

    /*!
      \brief Default constructor. Called from Arduino sketch when creating new LoRa instance.
      \param mod Instance of Module that will be used to communicate with the %LoRa chip.
    */
    SX1278(Module* mod); // cppcheck-suppress noExplicitConstructor

    // basic methods

    /*!
      \brief %LoRa modem initialization method. Must be called at least once from Arduino sketch to initialize the module.
      \param freq Carrier frequency in MHz. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param bw %LoRa link bandwidth in kHz. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
      \param sf %LoRa link spreading factor. Allowed values range from 6 to 12.
      \param cr %LoRa link coding rate denominator. Allowed values range from 5 to 8.
      \param syncWord %LoRa sync word. Can be used to distinguish different networks. Note that value 0x34 is reserved for LoRaWAN networks.
      \param power Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
      \param preambleLength Length of %LoRa transmission preamble in symbols. The actual preamble length is 4.25 symbols longer than the set number.
      Allowed values range from 6 to 65535.
      \param gain Gain of receiver LNA (low-noise amplifier). Can be set to any integer in range 1 to 6 where 1 is the highest gain.
      Set to 0 to enable automatic gain control (recommended).
      \returns \ref status_codes
    */
    virtual int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = RADIOLIB_SX127X_SYNC_WORD, int8_t power = 10, uint16_t preambleLength = 8, uint8_t gain = 0);

    /*!
      \brief FSK modem initialization method. Must be called at least once from Arduino sketch to initialize the module.
      \param freq Carrier frequency in MHz. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param br Bit rate of the FSK transmission in kbps (kilobits per second). Allowed values range from 1.2 to 300.0 kbps.
      \param freqDev Frequency deviation of the FSK transmission in kHz. Allowed values range from 0.6 to 200.0 kHz.
      Note that the allowed range changes based on bit rate setting, so that the condition FreqDev + BitRate/2 <= 250 kHz is always met.
      \param rxBw Receiver bandwidth in kHz. Allowed values are 2.6, 3.1, 3.9, 5.2, 6.3, 7.8, 10.4, 12.5, 15.6, 20.8, 25, 31.3, 41.7, 50, 62.5, 83.3, 100, 125, 166.7, 200 and 250 kHz.
      \param power Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
      \param preambleLength Length of FSK preamble in bits.
      \param enableOOK Use OOK modulation instead of FSK.
      \returns \ref status_codes
    */
    virtual int16_t beginFSK(float freq = 434.0, float br = 4.8, float freqDev = 5.0, float rxBw = 125.0, int8_t power = 10, uint16_t preambleLength = 16, bool enableOOK = false);

    /*!
      \brief Reset method. Will reset the chip to the default state using RST pin.
    */
    void reset() override;

    // configuration methods

    /*!
      \brief Sets carrier frequency. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param freq Carrier frequency to be set in MHz.
      \returns \ref status_codes
    */
    int16_t setFrequency(float freq) override;

    /*!
      \brief Sets %LoRa link bandwidth. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz. Only available in %LoRa mode.
      \param bw %LoRa link bandwidth to be set in kHz.
      \returns \ref status_codes
    */
    int16_t setBandwidth(float bw);

    /*!
      \brief Sets %LoRa link spreading factor. Allowed values range from 6 to 12. Only available in %LoRa mode.
      \param sf %LoRa link spreading factor to be set.
      \returns \ref status_codes
    */
    virtual int16_t setSpreadingFactor(uint8_t sf);

    /*!
      \brief Sets %LoRa link coding rate denominator. Allowed values range from 5 to 8. Only available in %LoRa mode.
      \param cr %LoRa link coding rate denominator to be set.
      \returns \ref status_codes
    */
    int16_t setCodingRate(uint8_t cr);

    /*!
      \brief Sets FSK bit rate. Allowed values range from 0.5 to 300 kbps. Only available in FSK mode.
      \param br Bit rate to be set (in kbps).
      \returns \ref status_codes
    */
    int16_t setBitRate(float br) override;
        
    /*!
      \brief Set data.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    int16_t setDataRate(DataRate_t dr) override;
    
    /*!
      \brief Check the data rate can be configured by this module.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    int16_t checkDataRate(DataRate_t dr) override;

    /*!
      \brief Sets transmission output power. Allowed values range from -4 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin).
      High power +20 dBm operation is also supported, on the PA_BOOST pin. Defaults to PA_BOOST.
      \param power Transmission output power in dBm.
      \returns \ref status_codes
    */
    int16_t setOutputPower(int8_t power) override;

    /*!
      \brief Sets transmission output power. Allowed values range from -4 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin).
      High power +20 dBm operation is also supported, on the PA_BOOST pin.
      \param power Transmission output power in dBm.
      \param forceRfo Whether to force using the RFO pin for the RF output (true)
      or to leave the selection up to user (false) based on power output.
      \returns \ref status_codes
    */
    int16_t setOutputPower(int8_t power, bool forceRfo);

    /*!
      \brief Check if output power is configurable.
      This method is needed for compatibility with PhysicalLayer::checkOutputPower.
      \param power Output power in dBm, assumes PA_BOOST pin.
      \param clipped Clipped output power value to what is possible within the module's range.
      \returns \ref status_codes
    */
    int16_t checkOutputPower(int8_t power, int8_t* clipped) override;

    /*!
      \brief Check if output power is configurable.
      \param power Output power in dBm.
      \param clipped Clipped output power value to what is possible within the module's range.
      \param useRfo Whether to use the RFO (true) or the PA_BOOST (false) pin for the RF output.
      \returns \ref status_codes
    */
    int16_t checkOutputPower(int8_t power, int8_t* clipped, bool useRfo);

    /*!
      \brief Sets gain of receiver LNA (low-noise amplifier). Can be set to any integer in range 1 to 6 where 1 is the highest gain.
      Set to 0 to enable automatic gain control (recommended).
      \param gain Gain of receiver LNA (low-noise amplifier) to be set.
      \returns \ref status_codes
    */
    int16_t setGain(uint8_t gain);

    /*!
      \brief Sets Gaussian filter bandwidth-time product that will be used for data shaping. Only available in FSK mode with FSK modulation.
      Allowed values are RADIOLIB_SHAPING_0_3, RADIOLIB_SHAPING_0_5 or RADIOLIB_SHAPING_1_0. Set to RADIOLIB_SHAPING_NONE to disable data shaping.
      \param sh Gaussian shaping bandwidth-time product that will be used for data shaping
      \returns \ref status_codes
    */
    int16_t setDataShaping(uint8_t sh) override;

    /*!
      \brief Sets filter cutoff frequency that will be used for data shaping.
      Allowed values are 1 for frequency equal to bit rate and 2 for frequency equal to 2x bit rate. Set to 0 to disable data shaping.
      Only available in FSK mode with OOK modulation.
      \param sh Cutoff frequency that will be used for data shaping
      \returns \ref status_codes
    */
    int16_t setDataShapingOOK(uint8_t sh);

    /*!
      \brief Gets recorded signal strength indicator.
      Overload with packet mode enabled for PhysicalLayer compatibility.
      \returns RSSI value in dBm.
    */
    float getRSSI() override;

    /*!
      \brief Gets recorded signal strength indicator.
      \param packet Whether to read last packet RSSI, or the current value. LoRa mode only, ignored for FSK.
      \param skipReceive Set to true to skip putting radio in receive mode for the RSSI measurement in FSK/OOK mode.
      \returns RSSI value in dBm.
    */
    float getRSSI(bool packet, bool skipReceive = false);

    /*!
      \brief Enables/disables CRC check of received packets.
      \param enable Enable (true) or disable (false) CRC.
      \param mode Set CRC mode to SX127X_CRC_WHITENING_TYPE_CCITT for CCITT, polynomial X16 + X12 + X5 + 1 (false)
      or SX127X_CRC_WHITENING_TYPE_IBM for IBM, polynomial X16 + X15 + X2 + 1 (true). Only valid in FSK mode.
      \returns \ref status_codes
    */
    int16_t setCRC(bool enable, bool mode = false);

    /*!
      \brief Forces LoRa low data rate optimization. Only available in LoRa mode. After calling this method,
      LDRO will always be set to the provided value, regardless of symbol length.
      To re-enable automatic LDRO configuration, call SX1278::autoLDRO()
      \param enable Force LDRO to be always enabled (true) or disabled (false).
      \returns \ref status_codes
    */
    int16_t forceLDRO(bool enable);

    /*!
      \brief Re-enables automatic LDRO configuration. Only available in LoRa mode. After calling this method,
      LDRO will be enabled automatically when symbol length exceeds 16 ms.
      \returns \ref status_codes
    */
    int16_t autoLDRO();

    /*!
      \brief Set implicit header mode for future reception/transmission. Required for spreading factor 6.
      \param len Payload length in bytes.
      \returns \ref status_codes
    */
    int16_t implicitHeader(size_t len);

    /*!
      \brief Set explicit header mode for future reception/transmission.
      \returns \ref status_codes
    */
    int16_t explicitHeader();
    
    /*!
      \brief Set modem for the radio to use. Will perform full reset and reconfigure the radio
      using its default parameters.
      \param modem Modem type to set - FSK or LoRa.
      \returns \ref status_codes
    */
    int16_t setModem(ModemType_t modem) override;

#if !RADIOLIB_GODMODE
  protected:
#endif
    int16_t setBandwidthRaw(uint8_t newBandwidth);
    int16_t setSpreadingFactorRaw(uint8_t newSpreadingFactor);
    int16_t setCodingRateRaw(uint8_t newCodingRate);

    int16_t configFSK() override;
    void errataFix(bool rx) override;

#if !RADIOLIB_GODMODE
  private:
#endif
    bool ldroAuto = true;
    bool ldroEnabled = false;

};

/*!
  \class RFM98
  \brief Only exists as alias for SX1278, since there seems to be no difference between %RFM98 and %SX1278 modules.
*/
RADIOLIB_TYPE_ALIAS(SX1278, RFM98)

#endif

#endif

//cloned from SX1278.h
// SX1278 specific register map
#define RADIOLIB_SX1278_REG_MODEM_CONFIG_3                      0x26
#define RADIOLIB_SX1278_REG_PLL_HOP                             0x44
#define RADIOLIB_SX1278_REG_TCXO                                0x4B
#define RADIOLIB_SX1278_REG_PA_DAC                              0x4D
#define RADIOLIB_SX1278_REG_FORMER_TEMP                         0x5B
#define RADIOLIB_SX1278_REG_BIT_RATE_FRAC                       0x5D
#define RADIOLIB_SX1278_REG_AGC_REF                             0x61
#define RADIOLIB_SX1278_REG_AGC_THRESH_1                        0x62
#define RADIOLIB_SX1278_REG_AGC_THRESH_2                        0x63
#define RADIOLIB_SX1278_REG_AGC_THRESH_3                        0x64
#define RADIOLIB_SX1278_REG_PLL                                 0x70

// SX1278 LoRa modem settings
// RADIOLIB_SX1278_REG_OP_MODE                                                MSB   LSB   DESCRIPTION
#define RADIOLIB_SX1278_HIGH_FREQ                               0b00000000  //  3     3   access HF test registers
#define RADIOLIB_SX1278_LOW_FREQ                                0b00001000  //  3     3   access LF test registers

// RADIOLIB_SX1278_REG_FRF_MSB + REG_FRF_MID + REG_FRF_LSB
#define RADIOLIB_SX1278_FRF_MSB                                 0x6C        //  7     0   carrier frequency setting: f_RF = (F(XOSC) * FRF)/2^19
#define RADIOLIB_SX1278_FRF_MID                                 0x80        //  7     0       where F(XOSC) = 32 MHz
#define RADIOLIB_SX1278_FRF_LSB                                 0x00        //  7     0             FRF = 3 byte value of FRF registers

// RADIOLIB_SX1278_REG_PA_CONFIG
#define RADIOLIB_SX1278_MAX_POWER                               0b01110000  //  6     4   max power: P_max = 10.8 + 0.6*MAX_POWER [dBm]; P_max(MAX_POWER = 0b111) = 15 dBm
#define RADIOLIB_SX1278_LOW_POWER                               0b00100000  //  6     4

// RADIOLIB_SX1278_REG_LNA
#define RADIOLIB_SX1278_LNA_BOOST_LF_OFF                        0b00000000  //  4     3   default LNA current

// SX127X_REG_MODEM_CONFIG_1
#define RADIOLIB_SX1278_BW_7_80_KHZ                             0b00000000  //  7     4   bandwidth:  7.80 kHz
#define RADIOLIB_SX1278_BW_10_40_KHZ                            0b00010000  //  7     4               10.40 kHz
#define RADIOLIB_SX1278_BW_15_60_KHZ                            0b00100000  //  7     4               15.60 kHz
#define RADIOLIB_SX1278_BW_20_80_KHZ                            0b00110000  //  7     4               20.80 kHz
#define RADIOLIB_SX1278_BW_31_25_KHZ                            0b01000000  //  7     4               31.25 kHz
#define RADIOLIB_SX1278_BW_41_70_KHZ                            0b01010000  //  7     4               41.70 kHz
#define RADIOLIB_SX1278_BW_62_50_KHZ                            0b01100000  //  7     4               62.50 kHz
#define RADIOLIB_SX1278_BW_125_00_KHZ                           0b01110000  //  7     4               125.00 kHz
#define RADIOLIB_SX1278_BW_250_00_KHZ                           0b10000000  //  7     4               250.00 kHz
#define RADIOLIB_SX1278_BW_500_00_KHZ                           0b10010000  //  7     4               500.00 kHz
#define RADIOLIB_SX1278_CR_4_5                                  0b00000010  //  3     1   error coding rate:  4/5
#define RADIOLIB_SX1278_CR_4_6                                  0b00000100  //  3     1                       4/6
#define RADIOLIB_SX1278_CR_4_7                                  0b00000110  //  3     1                       4/7
#define RADIOLIB_SX1278_CR_4_8                                  0b00001000  //  3     1                       4/8
#define RADIOLIB_SX1278_HEADER_EXPL_MODE                        0b00000000  //  0     0   explicit header mode
#define RADIOLIB_SX1278_HEADER_IMPL_MODE                        0b00000001  //  0     0   implicit header mode

// SX127X_REG_MODEM_CONFIG_2
#define RADIOLIB_SX1278_RX_CRC_MODE_OFF                         0b00000000  //  2     2   CRC disabled
#define RADIOLIB_SX1278_RX_CRC_MODE_ON                          0b00000100  //  2     2   CRC enabled

// RADIOLIB_SX1278_REG_MODEM_CONFIG_3
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_OFF                   0b00000000  //  3     3   low data rate optimization disabled
#define RADIOLIB_SX1278_LOW_DATA_RATE_OPT_ON                    0b00001000  //  3     3   low data rate optimization enabled
#define RADIOLIB_SX1278_AGC_AUTO_OFF                            0b00000000  //  2     2   LNA gain set by REG_LNA
#define RADIOLIB_SX1278_AGC_AUTO_ON                             0b00000100  //  2     2   LNA gain set by internal AGC loop

// SX127X_REG_VERSION
#define RADIOLIB_SX1278_CHIP_VERSION                            0x12  // this is the "official" version listed in datasheet
#define RADIOLIB_SX1278_CHIP_VERSION_ALT                        0x13  // appears sometimes 
#define RADIOLIB_SX1278_CHIP_VERSION_RFM9X                      0x11  // this value is used for the RFM9x

// SX1278 FSK modem settings
// SX127X_REG_PA_RAMP
#define RADIOLIB_SX1278_NO_SHAPING                              0b00000000  //  6     5   data shaping: no shaping (default)
#define RADIOLIB_SX1278_FSK_GAUSSIAN_1_0                        0b00100000  //  6     5                 FSK modulation Gaussian filter, BT = 1.0
#define RADIOLIB_SX1278_FSK_GAUSSIAN_0_5                        0b01000000  //  6     5                 FSK modulation Gaussian filter, BT = 0.5
#define RADIOLIB_SX1278_FSK_GAUSSIAN_0_3                        0b01100000  //  6     5                 FSK modulation Gaussian filter, BT = 0.3
#define RADIOLIB_SX1278_OOK_FILTER_BR                           0b00100000  //  6     5                 OOK modulation filter, f_cutoff = BR
#define RADIOLIB_SX1278_OOK_FILTER_2BR                          0b01000000  //  6     5                 OOK modulation filter, f_cutoff = 2*BR

// RADIOLIB_SX1278_REG_AGC_REF
#define RADIOLIB_SX1278_AGC_REFERENCE_LEVEL_LF                  0x19        //  5     0   floor reference for AGC thresholds: AgcRef = -174 + 10*log(2*RxBw) + 8 + AGC_REFERENCE_LEVEL [dBm]: below 525 MHz
#define RADIOLIB_SX1278_AGC_REFERENCE_LEVEL_HF                  0x1C        //  5     0                                                                                                         above 779 MHz

// RADIOLIB_SX1278_REG_AGC_THRESH_1
#define RADIOLIB_SX1278_AGC_STEP_1_LF                           0x0C        //  4     0   1st AGC threshold: below 525 MHz
#define RADIOLIB_SX1278_AGC_STEP_1_HF                           0x0E        //  4     0                      above 779 MHz

// RADIOLIB_SX1278_REG_AGC_THRESH_2
#define RADIOLIB_SX1278_AGC_STEP_2_LF                           0x40        //  7     4   2nd AGC threshold: below 525 MHz
#define RADIOLIB_SX1278_AGC_STEP_2_HF                           0x50        //  7     4                      above 779 MHz
#define RADIOLIB_SX1278_AGC_STEP_3                              0x0B        //  3     0   3rd AGC threshold

// RADIOLIB_SX1278_REG_AGC_THRESH_3
#define RADIOLIB_SX1278_AGC_STEP_4                              0xC0        //  7     4   4th AGC threshold
#define RADIOLIB_SX1278_AGC_STEP_5                              0x0C        //  4     0   5th AGC threshold


/*!
  \class SX1278
  \brief Derived class for %SX1278 modules. Also used as base class for SX1276, SX1277, SX1279, RFM95 and RFM96.
  All of these modules use the same basic hardware and only differ in parameter ranges (and names).
*/
class SX1278: public SX127x {
  public:

    // constructor

    /*!
      \brief Default constructor. Called from Arduino sketch when creating new LoRa instance.
      \param mod Instance of Module that will be used to communicate with the %LoRa chip.
    */
    SX1278(Module* mod); // cppcheck-suppress noExplicitConstructor

    // basic methods

    /*!
      \brief %LoRa modem initialization method. Must be called at least once from Arduino sketch to initialize the module.
      \param freq Carrier frequency in MHz. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param bw %LoRa link bandwidth in kHz. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz.
      \param sf %LoRa link spreading factor. Allowed values range from 6 to 12.
      \param cr %LoRa link coding rate denominator. Allowed values range from 5 to 8.
      \param syncWord %LoRa sync word. Can be used to distinguish different networks. Note that value 0x34 is reserved for LoRaWAN networks.
      \param power Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
      \param preambleLength Length of %LoRa transmission preamble in symbols. The actual preamble length is 4.25 symbols longer than the set number.
      Allowed values range from 6 to 65535.
      \param gain Gain of receiver LNA (low-noise amplifier). Can be set to any integer in range 1 to 6 where 1 is the highest gain.
      Set to 0 to enable automatic gain control (recommended).
      \returns \ref status_codes
    */
    virtual int16_t begin(float freq = 434.0, float bw = 125.0, uint8_t sf = 9, uint8_t cr = 7, uint8_t syncWord = RADIOLIB_SX127X_SYNC_WORD, int8_t power = 10, uint16_t preambleLength = 8, uint8_t gain = 0);

    /*!
      \brief FSK modem initialization method. Must be called at least once from Arduino sketch to initialize the module.
      \param freq Carrier frequency in MHz. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param br Bit rate of the FSK transmission in kbps (kilobits per second). Allowed values range from 1.2 to 300.0 kbps.
      \param freqDev Frequency deviation of the FSK transmission in kHz. Allowed values range from 0.6 to 200.0 kHz.
      Note that the allowed range changes based on bit rate setting, so that the condition FreqDev + BitRate/2 <= 250 kHz is always met.
      \param rxBw Receiver bandwidth in kHz. Allowed values are 2.6, 3.1, 3.9, 5.2, 6.3, 7.8, 10.4, 12.5, 15.6, 20.8, 25, 31.3, 41.7, 50, 62.5, 83.3, 100, 125, 166.7, 200 and 250 kHz.
      \param power Transmission output power in dBm. Allowed values range from 2 to 17 dBm.
      \param preambleLength Length of FSK preamble in bits.
      \param enableOOK Use OOK modulation instead of FSK.
      \returns \ref status_codes
    */
    virtual int16_t beginFSK(float freq = 434.0, float br = 4.8, float freqDev = 5.0, float rxBw = 125.0, int8_t power = 10, uint16_t preambleLength = 16, bool enableOOK = false);

    /*!
      \brief Reset method. Will reset the chip to the default state using RST pin.
    */
    void reset() override;

    // configuration methods

    /*!
      \brief Sets carrier frequency. Allowed values range from 137.0 MHz to 525.0 MHz.
      \param freq Carrier frequency to be set in MHz.
      \returns \ref status_codes
    */
    int16_t setFrequency(float freq) override;

    /*!
      \brief Sets %LoRa link bandwidth. Allowed values are 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 and 500 kHz. Only available in %LoRa mode.
      \param bw %LoRa link bandwidth to be set in kHz.
      \returns \ref status_codes
    */
    int16_t setBandwidth(float bw);

    /*!
      \brief Sets %LoRa link spreading factor. Allowed values range from 6 to 12. Only available in %LoRa mode.
      \param sf %LoRa link spreading factor to be set.
      \returns \ref status_codes
    */
    virtual int16_t setSpreadingFactor(uint8_t sf);

    /*!
      \brief Sets %LoRa link coding rate denominator. Allowed values range from 5 to 8. Only available in %LoRa mode.
      \param cr %LoRa link coding rate denominator to be set.
      \returns \ref status_codes
    */
    int16_t setCodingRate(uint8_t cr);

    /*!
      \brief Sets FSK bit rate. Allowed values range from 0.5 to 300 kbps. Only available in FSK mode.
      \param br Bit rate to be set (in kbps).
      \returns \ref status_codes
    */
    int16_t setBitRate(float br) override;
        
    /*!
      \brief Set data.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    int16_t setDataRate(DataRate_t dr) override;
    
    /*!
      \brief Check the data rate can be configured by this module.
      \param dr Data rate struct. Interpretation depends on currently active modem (FSK or LoRa).
      \returns \ref status_codes
    */
    int16_t checkDataRate(DataRate_t dr) override;

    /*!
      \brief Sets transmission output power. Allowed values range from -4 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin).
      High power +20 dBm operation is also supported, on the PA_BOOST pin. Defaults to PA_BOOST.
      \param power Transmission output power in dBm.
      \returns \ref status_codes
    */
    int16_t setOutputPower(int8_t power) override;

    /*!
      \brief Sets transmission output power. Allowed values range from -4 to 15 dBm (RFO pin) or +2 to +17 dBm (PA_BOOST pin).
      High power +20 dBm operation is also supported, on the PA_BOOST pin.
      \param power Transmission output power in dBm.
      \param forceRfo Whether to force using the RFO pin for the RF output (true)
      or to leave the selection up to user (false) based on power output.
      \returns \ref status_codes
    */
    int16_t setOutputPower(int8_t power, bool forceRfo);

    /*!
      \brief Check if output power is configurable.
      This method is needed for compatibility with PhysicalLayer::checkOutputPower.
      \param power Output power in dBm, assumes PA_BOOST pin.
      \param clipped Clipped output power value to what is possible within the module's range.
      \returns \ref status_codes
    */
    int16_t checkOutputPower(int8_t power, int8_t* clipped) override;

    /*!
      \brief Check if output power is configurable.
      \param power Output power in dBm.
      \param clipped Clipped output power value to what is possible within the module's range.
      \param useRfo Whether to use the RFO (true) or the PA_BOOST (false) pin for the RF output.
      \returns \ref status_codes
    */
    int16_t checkOutputPower(int8_t power, int8_t* clipped, bool useRfo);

    /*!
      \brief Sets gain of receiver LNA (low-noise amplifier). Can be set to any integer in range 1 to 6 where 1 is the highest gain.
      Set to 0 to enable automatic gain control (recommended).
      \param gain Gain of receiver LNA (low-noise amplifier) to be set.
      \returns \ref status_codes
    */
    int16_t setGain(uint8_t gain);

    /*!
      \brief Sets Gaussian filter bandwidth-time product that will be used for data shaping. Only available in FSK mode with FSK modulation.
      Allowed values are RADIOLIB_SHAPING_0_3, RADIOLIB_SHAPING_0_5 or RADIOLIB_SHAPING_1_0. Set to RADIOLIB_SHAPING_NONE to disable data shaping.
      \param sh Gaussian shaping bandwidth-time product that will be used for data shaping
      \returns \ref status_codes
    */
    int16_t setDataShaping(uint8_t sh) override;

    /*!
      \brief Sets filter cutoff frequency that will be used for data shaping.
      Allowed values are 1 for frequency equal to bit rate and 2 for frequency equal to 2x bit rate. Set to 0 to disable data shaping.
      Only available in FSK mode with OOK modulation.
      \param sh Cutoff frequency that will be used for data shaping
      \returns \ref status_codes
    */
    int16_t setDataShapingOOK(uint8_t sh);

    /*!
      \brief Gets recorded signal strength indicator.
      Overload with packet mode enabled for PhysicalLayer compatibility.
      \returns RSSI value in dBm.
    */
    float getRSSI() override;

    /*!
      \brief Gets recorded signal strength indicator.
      \param packet Whether to read last packet RSSI, or the current value. LoRa mode only, ignored for FSK.
      \param skipReceive Set to true to skip putting radio in receive mode for the RSSI measurement in FSK/OOK mode.
      \returns RSSI value in dBm.
    */
    float getRSSI(bool packet, bool skipReceive = false);

    /*!
      \brief Enables/disables CRC check of received packets.
      \param enable Enable (true) or disable (false) CRC.
      \param mode Set CRC mode to SX127X_CRC_WHITENING_TYPE_CCITT for CCITT, polynomial X16 + X12 + X5 + 1 (false)
      or SX127X_CRC_WHITENING_TYPE_IBM for IBM, polynomial X16 + X15 + X2 + 1 (true). Only valid in FSK mode.
      \returns \ref status_codes
    */
    int16_t setCRC(bool enable, bool mode = false);

    /*!
      \brief Forces LoRa low data rate optimization. Only available in LoRa mode. After calling this method,
      LDRO will always be set to the provided value, regardless of symbol length.
      To re-enable automatic LDRO configuration, call SX1278::autoLDRO()
      \param enable Force LDRO to be always enabled (true) or disabled (false).
      \returns \ref status_codes
    */
    int16_t forceLDRO(bool enable);

    /*!
      \brief Re-enables automatic LDRO configuration. Only available in LoRa mode. After calling this method,
      LDRO will be enabled automatically when symbol length exceeds 16 ms.
      \returns \ref status_codes
    */
    int16_t autoLDRO();

    /*!
      \brief Set implicit header mode for future reception/transmission. Required for spreading factor 6.
      \param len Payload length in bytes.
      \returns \ref status_codes
    */
    int16_t implicitHeader(size_t len);

    /*!
      \brief Set explicit header mode for future reception/transmission.
      \returns \ref status_codes
    */
    int16_t explicitHeader();
    
    /*!
      \brief Set modem for the radio to use. Will perform full reset and reconfigure the radio
      using its default parameters.
      \param modem Modem type to set - FSK or LoRa.
      \returns \ref status_codes
    */
    int16_t setModem(ModemType_t modem) override;

#if !RADIOLIB_GODMODE
  protected:
#endif
    int16_t setBandwidthRaw(uint8_t newBandwidth);
    int16_t setSpreadingFactorRaw(uint8_t newSpreadingFactor);
    int16_t setCodingRateRaw(uint8_t newCodingRate);

    int16_t configFSK() override;
    void errataFix(bool rx) override;

#if !RADIOLIB_GODMODE
  private:
#endif
    bool ldroAuto = true;
    bool ldroEnabled = false;

};

/*!
  \class RFM98
  \brief Only exists as alias for SX1278, since there seems to be no difference between %RFM98 and %SX1278 modules.
*/
RADIOLIB_TYPE_ALIAS(SX1278, RFM98)

#endif

#endif


