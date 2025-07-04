/*
  RadioLib SX127x FSK Modem Example

  This example shows how to use FSK modem in SX127x chips.

  NOTE: The sketch below is just a guide on how to use
        FSK modem, so this code should not be run directly!
        Instead, modify the other examples to use FSK
        modem and use the appropriate configuration
        methods.

  For default module settings, see the wiki page
  https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---fsk-modem

  For full API reference, see the GitHub Pages
  https://jgromes.github.io/RadioLib/
*/

// include the library
#include </src/RadioLib.h>
#include <Spresense.h>

// SX1278 has the following connections:(Arduino MKR WAN 1310)
// NSS pin:   7
// DIO0 pin:  1
// RESET pin: 6
// DIO1 pin:  0


// SX1278 has the following connections:(Spresense)
// NSS pin:   4
// DIO0 pin:  2
// RESET pin: 9
// DIO1 pin:  3

int device = 1; // 0 for MKR WAN 1310, 1 for Spresense

switch (device) {
  case 0:
    Serial.println(F("Using MKR WAN 1310 pinout!"));
    SX1278 radio = new Module(7, 1, 6, 0);
    break;
  case 1:
    Serial.println(F("Using Spresense pinout!"));
    SX1278 radio = new Module(4, 2, 9, 3);
    break;
  default:
    Serial.println(F("Unknown device, using MKR WAN 1310 pinout!"));
    device = 0;
}


// or detect the pinout automatically using RadioBoards
// https://github.com/radiolib-org/RadioBoards
/*
#define RADIO_BOARD_AUTO
#include <RadioBoards.h>
Radio radio = new RadioModule();
*/

//Global variables for FSK settings
float freq, bitRate, freqDev, rxBw, outputPower;
int currentLimit, syncWord;

void setup() {
  delay(2000);

  Serial.begin(115200);

  // initialize SX1278 FSK modem with default settings
  Serial.print(F("[SX1278] Initializing ... "));
  int state = radio.beginFSK();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // if needed, you can switch between LoRa and FSK modes
  //
  // radio.begin()       start LoRa mode (and disable FSK)
  // radio.beginFSK()    start FSK mode (and disable LoRa)

  // the following settings can also
  // be modified at run-time
    state = radio.setFrequency(437.05);
    state = radio.setBitRate(1.2);
    state = radio.setFrequencyDeviation(4.0);
    state = radio.setRxBandwidth(10.4);
    state = radio.setOutputPower(17.0);
    state = radio.setCurrentLimit(100);
    state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                          0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, 8);

    showSettings();
    
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to set configuration, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
/*
  // FSK modulation can be changed to OOK
  // NOTE: When using OOK, the maximum bit rate is only 32.768 kbps!
  //       Also, data shaping changes from Gaussian filter to
  //       simple filter with cutoff frequency. Make sure to call
  //       setDataShapingOOK() to set the correct shaping!
  state = radio.setOOK(true);
  state = radio.setDataShapingOOK(1);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("Unable to change modulation, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }
*/
  #warning "This sketch is just an API guide! Read the note at line 6."

}
void loop() {
  if (Serial.available() > 0)
    {
        String input = Serial.readString();
        input.trim();
        handleInput(input);
    }

    receivePacket();
}

//Handle user input from the Serial Monitor
void handleInput(const String &input)// シリアルモニタからの入力を処理する関数
{
    if (input == "1")
    {
        transmit();
    }
    else if (input == "2")
    {
        transmitLoop();
    }
    else if (input == "3")
    {
        Serial.println("no ongoing loop to stop");
    }
    else if (input == "4")
    {
        setStateManual();
    }
    else
    {
        Serial.println("Invalid Input");
    }
}

// transmit FSK packet
void transmit() {
  // FSK modem can use the same transmit/receive methods
  // as the LoRa modem, even their interrupt-driven versions
  // NOTE: FSK modem maximum packet length is 63 bytes!

  // transmit FSK packet
  int state = radio.transmit("Yui Project");
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                      0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  showError();

  delay(1000);

}

// transmit FSK packet in a loop
void transmitLoop() {
    // FSK modem can use the same transmit/receive methods
    // as the LoRa modem, even their interrupt-driven versions
    // NOTE: FSK modem maximum packet length is 63 bytes!

    // transmit FSK packet
    while(1){
    int state = radio.transmit("Yui Project");
    Serial.println("Yui Project");

    if (Serial.available() > 0)
        {
            String input = Serial.readString();
            input.trim();
            if (input == "3")
            {
                break;
            }
        }
    }
    /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
    */

    showError();

    delay(1000);

}

// receive FSK packet
void receive() {
    // FSK modem can use the same transmit/receive methods
    // as the LoRa modem, even their interrupt-driven versions
    // NOTE: FSK modem maximum packet length is 63 bytes!

    // receive FSK packet
    String str;
    int state = radio.receive(str);
    /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
    */
    showError();

    delay(10);

}

// set FSK modem parameters 
void setState(float freq, float bitRate, float freqDev, float rxBw, float outputPower, int currentLimit, int syncWord) {
    
    state = radio.setFrequency(freq);
    state = radio.setBitRate(bitRate);
    state = radio.setFrequencyDeviation(freqDev);
    state = radio.setRxBandwidth(rxBw);
    state = radio.setOutputPower(outputPower);
    state = radio.setCurrentLimit(currentLimit);
    state = radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    uint8_t syncWord[] = {0x01, 0x23, 0x45, 0x67,
                        0x89, 0xAB, 0xCD, 0xEF};
    state = radio.setSyncWord(syncWord, syncWord);
  
}

// set FSK modem parameters manually
void setStateManual(float freq, float bitRate, float freqDev, float rxBw, float outputPower, int currentLimit, int syncWord) {
    Serial.println("[SX1278] Setting FSK parameters...");
    Serial.print("Enter number change settings");
    Serial.println(" (0: exit, 1: Frequency, 2: Bit Rate, 3: Frequency Deviation, 4: RX Bandwidth, 5: Output Power, 6: Current Limit, 7: Sync Word, 8: Show Settings):");

    while (true) {
        if (Serial.available() > 0) {
            String input = Serial.readString();
            input.trim();

            if (input == "0") {
                Serial.println("Exiting parameter change.");
                return; // Exit the loop
            } else if (input == "1") {
                Serial.print("Enter new frequency: example 437.05 (MHz): ");
                while (Serial.available() == 0);
                freq = Serial.parseFloat();
                freq = freq * 1000000; // Convert MHz to Hz
            } else if (input == "2") {
                Serial.print("Enter new bit rate: example 50 (kbps): ");
                while (Serial.available() == 0);
                bitRate = Serial.parseFloat();
            } else if (input == "3") {
                Serial.print("Enter new frequency deviation: example 5 (kHz): ");
                while (Serial.available() == 0);
                freqDev = Serial.parseFloat();
            } else if (input == "4") {
                Serial.print("Enter new RX bandwidth: example 125 (kHz): ");
                while (Serial.available() == 0);
                rxBw = Serial.parseFloat();
            } else if (input == "5") {
                Serial.print("Enter new output power: example 20 (dBm): ");
                while (Serial.available() == 0);
                outputPower = Serial.parseFloat();
            } else if (input == "6") {
                Serial.print("Enter new current limit: example 100 (mA): ");
                while (Serial.available() == 0);
                currentLimit = Serial.parseInt();
            } else if (input == "7") {
                Serial.print("Enter new sync word (hex): ");
                while (Serial.available() == 0);
                syncWord = Serial.parseInt();
            } else if (input == "8") {
                showSettings();
            } else {
                Serial.println("Invalid input");
                continue;
            }

            setState(freq, bitRate, freqDev, rxBw, outputPower, currentLimit, syncWord);
            Serial.println("FSK parameters updated.");
            showSettings(); // Show updated settings
            Serial.println("");
            Serial.print("Enter number change settings");
            Serial.println(" (0: exit, 1: Frequency, 2: Bit Rate, 3: Frequency Deviation, 4: RX Bandwidth, 5: Output Power, 6: Current Limit, 7: Sync Word, 8: Show Settings):");
        }
    }
  
}

// show current FSK settings
void showSettings() {
    Serial.println("[SX1278] Current FSK settings:");
    Serial.print("Frequency: ");
    Serial.print(freq / 1000000); // Convert Hz to MHz
    Serial.println(" MHz");
    Serial.print("Bit Rate: ");
    Serial.print(bitRate);
    Serial.println(" kbps");
    Serial.print("Frequency Deviation: ");
    Serial.print(freqDev);
    Serial.println(" kHz");
    Serial.print("RX Bandwidth: ");
    Serial.print(rxBw);
    Serial.println(" kHz");
    Serial.print("Output Power: ");
    Serial.print(outputPower);
    Serial.println(" dBm");
    Serial.print("Current Limit: ");
    Serial.print(currentLimit);
    Serial.println(" mA");
    Serial.print("Sync Word: ");
    Serial.println(syncWord, HEX);
}
// show error message based on the state
void showError(){
    if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("[SX1278] Packet transmitted successfully!"));
    } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    Serial.println(F("[SX1278] Packet too long!"));
    } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
    Serial.println(F("[SX1278] Timed out while transmitting!"));
    } else {
    Serial.println(F("[SX1278] Failed to transmit packet, code "));
    Serial.println(state);
    }
}