
// Arduino Library for Kangaroo
// Copyright (c) 2013-2014 Dimension Engineering LLC
// http://www.dimensionengineering.com/kangaroo

/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file utilizes XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <SoftwareSerial.h>
#include <Kangaroo.h>
#include <Printers.h>
#include <XBee.h>

/**
 * Variable definitions for XBee communication
 */

SoftwareSerial xbeePort(2,3);

XBee xbee = XBee();
XBeeResponse respone = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
uint8_t outgoingPayload[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
XBeeAddress64 addr64 = XBeeAddress64(0x00132A00, 0x403E0F30);
ZBTxRequest zbTx = ZBTxRequest(addr64, outgoingPayload, sizeof(outgoingPayload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
/**
 * Variable definitions for Kangaroo communication
 */

KangarooSerial kanga(Serial3);
KangarooChannel Drive(kanga, 'D');
KangarooChannel Turn(kanga, 'T');

struct moveData {
  int distance;
  int angle;
  int velocity;
  int rotVelocity;
};

union arrayToInt {
  byte array[4];
  uint32_t integer;
};

moveData newMove;
arrayToInt converter;

uint8_t err;

const int MOVING = 1;
const int NOT_MOVING = 0;
const int STARTED = 0;
const int NOT_STARTED = 1;
const int NO_CONTROL_ERROR = 0;
const int CONTROL_ERROR = 1;
const int CORRECT_MODE = 0;
const int WRONG_MODE = 1;
const int NO_TIMEOUT = 0;
const int SERIAL_TIMEOUT = 1;

const int DRIVE_MOVEMENT_STATUS = 0;
const int DRIVE_START_STATUS = 1;
const int DRIVE_CONTROL_ERROR_STATUS = 3;
const int DRIVE_MODE_STATUS = 4;
const int DRIVE_TIMEOUT_STATUS = 5;
const int TURN_MOVEMENT_STATUS = 6;
const int TURN_START_STATUS = 7;
const int TURN_CONTROL_ERROR_STATUS = 8;
const int TURN_MODE_STATUS = 9;
const int TURN_TIMEOUT_STATUS = 10;
const int POSITION = 11;
const int VELOCITY = 12;
const int ANGULAR_POSITION = 13;
const int ROTATIONAL_VELOCITY = 14;

void setup() {
  Serial.begin(9600); // for debugging
//  Serial1.begin(9600); // for testing with Fio v3
//  Serial2.begin(9600); // For XBee communication. Must manually connect these pins to the XBee on the shield.
  xbeePort.begin(9600);
  Serial3.begin(9600); // for Kangaroo communication.

//  xbee.setSerial(Serial1); //use Serial1 for testing with Fio v3; xbeePort for Mega

  err = Drive.start();
  parseError("Drive", "Start", err);
    
  err = Turn.start();
  parseError("Turn", "Start", err);

//  reportErrors();

  Drive.s(0);
  Turn.s(0);

  newMove.distance = 0;
  newMove.angle = 0;
  newMove.velocity = 0;
  newMove.rotVelocity = 0;

  outgoingPayload[POSITION] = 0;
  outgoingPayload[VELOCITY] = 0;
  outgoingPayload[ANGULAR_POSITION] = 0;
  outgoingPayload[ROTATIONAL_VELOCITY] = 0;

  delay(5000);
  Serial.println("Listening...");
}

void loop() {
  setMovementStatus();
  newMove = checkXbee();
  if(newMove.distance > 0 || newMove.angle > 0) {
    Drive.pi(100);
    Turn.pi(newMove.angle, newMove.rotVelocity);
  }
  delay(100);
}

void setMovementStatus() {
  if(Drive.getpi().done() || Turn.getpi().done()) {
    outgoingPayload[DRIVE_MOVEMENT_STATUS] = NOT_MOVING;
  } else {
    outgoingPayload[DRIVE_MOVEMENT_STATUS] = MOVING;
  }
}

boolean executeMove(moveData move) {

  return true;
}

moveData checkXbee() {
  moveData data;
  data.distance = 0;
  data.angle = 0;
  data.velocity = 0;
  data.rotVelocity = 0;
  byte payload[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  xbee.readPacket();
  if(xbee.getResponse().isAvailable()) {
    Serial.println("Got something.");
    if(xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(rx);
      Serial.println("Got a message!");
      for(int i = 0; i < rx.getDataLength(); i++) {
        payload[i] = rx.getData(i);
      }
    } else if(xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
      xbee.getResponse().getModemStatusResponse(msr);
      if(msr.getStatus() == ASSOCIATED) {
        //indicate associated
      } else if(msr.getStatus() == DISASSOCIATED) {
        //indicate not associated
      } else {
        //indicate unknown error
      }
    } else {
      //indicate unknown error
    }
  } else if(xbee.getResponse().isError()) {
    //indicate error
  }
  data.distance = convertInteger(payload, 0);
  data.angle = convertInteger(payload, 4);
  data.velocity = convertInteger(payload, 8);
  data.rotVelocity = convertInteger(payload, 12);
  return data;
}

int convertInteger(byte data[], int startIndex) {
  converter.array[3] = data[startIndex];
  converter.array[2] = data[startIndex + 1];
  converter.array[1] = data[startIndex + 2];
  converter.array[0] = data[startIndex + 3];
  return converter.integer;
}

void parseError(const char *channel, const char *command, uint8_t err) {
  switch(err) {
    case KANGAROO_NO_ERROR:
      if(channel == "Drive") {
        outgoingPayload[DRIVE_START_STATUS] = STARTED;
        outgoingPayload[DRIVE_CONTROL_ERROR_STATUS] = NO_CONTROL_ERROR;
        outgoingPayload[DRIVE_MODE_STATUS] = CORRECT_MODE;
        outgoingPayload[DRIVE_TIMEOUT_STATUS] = NO_TIMEOUT;
      } else if (channel == "Turn") {
        outgoingPayload[TURN_START_STATUS] = STARTED;
        outgoingPayload[TURN_CONTROL_ERROR_STATUS] = NO_CONTROL_ERROR;
        outgoingPayload[TURN_MODE_STATUS] = CORRECT_MODE;
        outgoingPayload[TURN_TIMEOUT_STATUS] = NO_TIMEOUT;
      }
    break;
    case KANGAROO_NOT_STARTED:
      if(channel == "Drive") {
        outgoingPayload[DRIVE_START_STATUS] = NOT_STARTED;
      } else if(channel == "Turn") {
        outgoingPayload[TURN_START_STATUS] = NOT_STARTED;
      }
    break;
    case KANGAROO_CONTROL_ERROR:
      if(channel == "Drive") {
        outgoingPayload[DRIVE_CONTROL_ERROR_STATUS] = CONTROL_ERROR;
      } else if (channel = "Turn") {
        outgoingPayload[TURN_CONTROL_ERROR_STATUS] = CONTROL_ERROR;
      }
    break;
    case KANGAROO_WRONG_MODE:
      if(channel == "Drive") {
        outgoingPayload[DRIVE_MODE_STATUS] = WRONG_MODE;
      } else if(channel == "Turn") {
        outgoingPayload[TURN_MODE_STATUS] = WRONG_MODE;
      }
    break;
    case KANGAROO_SERIAL_TIMEOUT:
      if(channel == "Drive") {
        outgoingPayload[DRIVE_TIMEOUT_STATUS] = SERIAL_TIMEOUT;
      } else if(channel == "Turn") {
        outgoingPayload[TURN_TIMEOUT_STATUS] = SERIAL_TIMEOUT;
      }
    break;
  }
}

