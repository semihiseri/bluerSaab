/*
 * C++ Class for handling CD changer emulator on SAAB I-Bus
 * Copyright (C) 2016  Karlis Veilands
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Created by: Karlis Veilands
 * Created on: Jun 4, 2015
 * Modified by: Karlis Veilands
 * Modified on: November 29, 2016
 */

#ifndef CDC_H
#define CDC_H

//#include <Arduino.h>


/**
 * Various constants used for SID text control
 * Abbreviations:
 *      IHU - Infotainment Head Unit
 *      SPA - SAAB Park Assist
 *      EMS - Engine Management System
 */

//#define NODE_APL_ADR                  0x11    // IHU
#define NODE_APL_ADR                    0x15    // "BlueSaab" custom
//#define NODE_APL_ADR                  0x21    // ECS
//#define NODE_APL_ADR                  0x1F    // SPA
#define NODE_SID_FUNCTION_ID            0x17    // "BlueSaab" custom
//#define NODE_SID_FUNCTION_ID          0x18    // SPA
//#define NODE_SID_FUNCTION_ID          0x19    // IHU
//#define NODE_SID_FUNCTION_ID          0x32    // ECS
#define NODE_DISPLAY_RESOURCE_REQ       0x345   // "BlueSaab" custom
//#define NODE_DISPLAY_RESOURCE_REQ     0x348   // IHU
//#define NODE_DISPLAY_RESOURCE_REQ     0x357   // SPA
//#define NODE_DISPLAY_RESOURCE_REQ     0x358   // ECS
#define NODE_WRITE_TEXT_ON_DISPLAY      0x325   // "BlueSaab" custom
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x328   // IHU
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x337   // SPA
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x33F   // ECS

/**
 * Other useful stuff
 */

#define MODULE_NAME                 "BlueSaab"
#define LAST_EVENT_IN_TIMEOUT       3000    // Milliseconds
#define NODE_STATUS_TX_MSG_SIZE     4       // Decimal; defines how many frames do we need to reply with to '6A1'

/**
 * TX frames:
 */

#define GENERAL_STATUS_CDC          0x3C8
#define NODE_STATUS_TX_CDC          0x6A2
#define SOUND_REQUEST               0x430

/**
 * RX frames:
 */

#define CDC_CONTROL                 0x3C0
#define DISPLAY_RESOURCE_GRANT      0x368
#define NODE_STATUS_RX_IHU          0x6A1
#define STEERING_WHEEL_BUTTONS      0x290

/**
 * Timer definitions:
 */

#define NODE_STATUS_TX_INTERVAL     140     // Replies to '6A1' request need to be sent with no more than 140ms interval; tolerances +/- 10%
#define CDC_STATUS_TX_BASETIME      950     // The CDC status frame must be sent periodically within this timeframe; tolerances +/- 10%
#define SID_CONTROL_TX_BASETIME     1000    // SID control/resource request frames needs to be sent within this timeframe; tolerances +/- 10%

/**
 * Class:
 */

//public:
void CDC_printCanTxFrame();
void CDC_printCanRxFrame();
void CDC_openCanBus();
void CDC_handleRxFrame();
void CDC_handleIhuButtons();
void CDC_handleSteeringWheelButtons();
void CDC_handleCdcStatus();
void CDC_sendCdcStatus(bool event, bool remote, bool cdcActive);
void CDC_sendDisplayRequest(bool sidWriteAccessWanted);
void CDC_sendCanFrame(int message_id, unsigned char *msg);
void CDC_writeTextOnDisplay(const char textIn[]);
void CDC_checkCanEvent(int frameElement);

/*class CDChandler {
};*/

void sendDisplayRequestOnTime(/* void* */);
void writeTextOnDisplayOnTime(/* void* */);

/**
 * Variables:
 */

//extern CDChandler CDC;

#endif
