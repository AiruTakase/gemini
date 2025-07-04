// This file insists on the use of the Spresense board with the RadioLib library for FSK communication.
#ifndef Spresense_H
#define Spresense_H

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
