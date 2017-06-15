/*
  SBUS signal generator
  May be used to:
  - check SBUS decoders
  - control DJI Phantom 2
  - convert PPM or PWM to SBUS (is not implemented here)

  This program for Arduino board reads a string in the format
  channel_number=channel_value<CR>
  channel_number - 1..18
  channel_value - 0..2047
  
  The SoftwareSerial runs on 57600. And RX pin 12, TX pin 13.
  SBUS is connected to the Arduino pin 1 with an inverter.
*/

#include <SoftwareSerial.h> //for PC communication (commands reading)
#include <string.h> // it is needed for subString function

SoftwareSerial myInvertedConn(12, 13, false);

unsigned int SBUS_Channel_Data[18];
byte SBUS_Packet_Data[25];
byte SBUS_Failsafe_Active = 0;
byte SBUS_Lost_Frame = 0;

byte buffer_updated = 0xff;

void setup()
{
  Serial.begin(100000, SERIAL_8E2); //The correct UART format initialization - SERIAL_8E2
  myInvertedConn.begin(57600);
  //initaial SBUS values
  for (uint8_t i=0; i <= 15; i++) SBUS_Channel_Data[i] = 1024;
  SBUS_Channel_Data[16] = 0;
  SBUS_Channel_Data[17] = 0;
  SBUS_Failsafe_Active = 0;
  SBUS_Lost_Frame = 0;
}

void SBUS_Build_Packet(void)
{
  uint8_t SBUS_chan_num;
  uint16_t SBUS_chan_shifter;
  uint8_t SBUS_packet_pos;
  uint8_t SBUS_packet_shifter;

  for (SBUS_packet_pos = 0; SBUS_packet_pos < 25; SBUS_packet_pos++) SBUS_Packet_Data[SBUS_packet_pos] = 0x00; //Zero out packet data

  SBUS_packet_pos = 0;
  SBUS_Packet_Data[SBUS_packet_pos] = 0x0F;  //Start Byte
  SBUS_packet_pos++;

  SBUS_packet_shifter = 0x01;

  for (SBUS_chan_num = 0; SBUS_chan_num < 16; SBUS_chan_num++)
  {
    for (SBUS_chan_shifter = 0x01; SBUS_chan_shifter < (1 << 11); SBUS_chan_shifter <<= 1)
    {
      if (SBUS_Channel_Data[SBUS_chan_num] & SBUS_chan_shifter)
        SBUS_Packet_Data[SBUS_packet_pos] |= SBUS_packet_shifter;

      SBUS_packet_shifter <<= 1;
      if (SBUS_packet_shifter == 0)
      {
        SBUS_packet_shifter = 0x01;
        SBUS_packet_pos++;       //Move to the next packet byte
      }
    }
  }
  if (SBUS_Channel_Data[16] > 1023) SBUS_Packet_Data[23] |= (1 << 0); //Any number above 1023 will set the digital servo bit
  if (SBUS_Channel_Data[17] > 1023) SBUS_Packet_Data[23] |= (1 << 1);
  if (SBUS_Lost_Frame != 0) SBUS_Packet_Data[23] |= (1 << 2);       //Any number above 0 will set the lost frame and failsafe bits
  if (SBUS_Failsafe_Active != 0) SBUS_Packet_Data[23] |= (1 << 3);
  SBUS_Packet_Data[24] = 0x00;  //End byte
}

void loop()
{
  //We read serial port, lookin for the command
  if (myInvertedConn.available() > 0) { //makes sure something is ready to be read
    static uint8_t chan_num = 0;
    static uint16_t chan_val = 0;
    static uint8_t now_val = 0;

    char inchar = myInvertedConn.read(); //assigns one byte (as serial.read()'s only input one byte at a time
    myInvertedConn.print(inchar);

    switch (inchar) {
      case '=':
        now_val = 0xFF;
        break;
      case '\r':
        if ((chan_num > 0) && (chan_num <= 18) && (chan_val < 2048) && now_val) {
          SBUS_Channel_Data[chan_num - 1] = chan_val;
          buffer_updated = 0xff;
        }
        chan_num = 0;
        chan_val = 0;
        now_val = 0;
        break;
      default:
        if (isDigit(inchar)) {
          if (now_val)
            chan_val = chan_val * 10 + (inchar - '0');
          else
            chan_num = chan_num * 10 + (inchar - '0');
        }
        else
        {
          chan_num = 0;
          chan_val = 0;
          now_val = 0;
        }
    }
  }

  if (buffer_updated == 0xFF) {
    for (uint8_t i = 0; i < 16; i++) {
      myInvertedConn.print(SBUS_Channel_Data[i], HEX);
      myInvertedConn.print(",");
    }

    myInvertedConn.println("Channels value");
    SBUS_Build_Packet(); //Build the SBUS packet with the values
    for (uint8_t i = 0; i < 25; i++) {
      myInvertedConn.print("0x");
      if (SBUS_Packet_Data[i] < 0x10)
        myInvertedConn.print("0");
      myInvertedConn.print(SBUS_Packet_Data[i], HEX);
      myInvertedConn.print(",");
    }
    myInvertedConn.println("Raw SBUS");
    buffer_updated = 0x00;
  }

  Serial.write(SBUS_Packet_Data, 25); //Send data to SBUS

  delay(7);//We need to wait before sending next 25 packets
}


