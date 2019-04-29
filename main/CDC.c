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
 * Modified by: Sam Thompson
 * Modified on: December 15, 2016
 */

//#include <Arduino.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "sdkconfig.h" // why is this necessary?

#include "bluetoothAudio.h"


#include "CAN.h"
#include "CDC.h"
//#include "MessageSender.h"
//#include "RN52handler.h"
//#include "Timer.h"

#define DEBUGMODE  1

#define CAN_FRAME_LENGTH	8
#define MESSAGE_COUNT		3

#define millis() xTaskGetTickCount()*portTICK_RATE_MS

/**
 * Variables:
 */

//MessageSender messageSender; // These classes don't exist in ESP32. So, get rid of them somehow!
//extern Timer time;


void sendCdcActiveStatus(void*);
void sendCdcPowerdownStatus(void*);
void *currentCdcCmd = NULL;
volatile unsigned long cdcStatusLastSendTime = 0;            // Timer used to ensure we send the CDC status frame in a timely manner
unsigned long lastIcomingEventTime = 0;                      // Timer used for determening if we should treat current event as, for example, a long press of a button
bool cdcActive = false;                                   // True while our module, the simulated CDC, is active
bool sidWriteAccessWanted = false;                        // True while we want to write on SID
volatile bool cdcStatusResendNeeded = false;              // True if an internal operation has triggered the need to send the CDC status frame as an event
volatile bool cdcStatusResendDueToCdcCommand = false;     // True if the need for sending the CDC status frame was triggered by CDC_CONTROL frame (IHU)
bool writeTextOnDisplayTimerActive = false;               // True while we are writing custom text on SID every SID_CONTROL_TX_BASETIME interval
int incomingEventCounter = 0;                                // Counter for incoming events to determine when we will treat the event, for example, as a long press of a button
int displayRequestTimerId = -1;
int writeTextOnDisplayTimerId = -1;
int currentNodeStatusTxTimerEvent = -1;
int textToSidTimer = -1;
unsigned char cdcPoweronCmd[NODE_STATUS_TX_MSG_SIZE][CAN_FRAME_LENGTH] = {
    {0x32,0x00,0x00,0x03,0x01,0x02,0x00,0x00},
    {0x42,0x00,0x00,0x22,0x00,0x00,0x00,0x00},
    {0x52,0x00,0x00,0x22,0x00,0x00,0x00,0x00},
    {0x62,0x00,0x00,0x22,0x00,0x00,0x00,0x00}
};
unsigned char cdcActiveCmd[NODE_STATUS_TX_MSG_SIZE] [CAN_FRAME_LENGTH] = {
    {0x32,0x00,0x00,0x16,0x01,0x02,0x00,0x00},
    {0x42,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
    {0x52,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
    {0x62,0x00,0x00,0x36,0x00,0x00,0x00,0x00},
};
unsigned char cdcPowerdownCmd[NODE_STATUS_TX_MSG_SIZE] [CAN_FRAME_LENGTH] = {
    {0x32,0x00,0x00,0x19,0x01,0x00,0x00,0x00},
    {0x42,0x00,0x00,0x38,0x01,0x00,0x00,0x00},
    {0x52,0x00,0x00,0x38,0x01,0x00,0x00,0x00},
    {0x62,0x00,0x00,0x38,0x01,0x00,0x00,0x00}
};

/* Format of SOUND_REQUEST frame:
 ID: SOUND_REQUEST
 [0]: Sent on basetime/event; 0 = Basetime; 80 = Event
 [1]: Type of sound
 [2-7]: Zeroed out; not in use
 */
unsigned char soundCmd[] = {0x80,0x04,0x00,0x00,0x00,0x00,0x00,0x00};

/**
 * DEBUG: Prints the CAN Tx frame to serial output
 */

void CDC_printCanTxFrame() {
#if (DEBUGMODE==1)
    printf("%x", CAN_TxMsg.id);
    printf(" Tx-> ");
    for (int i = 0; i < CAN_FRAME_LENGTH; i++) {
        printf("%x", CAN_TxMsg.data[i]);
        printf(" ");
    }
    printf("\n");
#endif
}

/**
 * DEBUG: Prints the CAN Rx frame to serial output
 */

void CDC_printCanRxFrame() {
//#if (DEBUGMODE==1)
    printf("%x", CAN_RxMsg.id);
    printf(" Rx-> ");
    for (int i = 0; i < CAN_FRAME_LENGTH; i++) {
        printf("%x", CAN_RxMsg.data[i]);
        printf(" ");
    }
    printf("\n");
//#endif
}

/**
 * Opens CAN bus for communication; SAAB I-Bus is 47.619Kbps
 */

void CDC_openCanBus() {
    CAN_begin(47);
    CAN_TxMsg.header.rtr = 0;
    CAN_TxMsg.header.length = CAN_FRAME_LENGTH;
}

/**
 * Handles an incoming (Rx)frame
 */

void CDC_handleRxFrame() {
    if (CAN_CheckNew()) {
        CAN_TxMsg.data[0]++;
        CAN_ReadFromDevice(&CAN_RxMsg);
        switch (CAN_RxMsg.id) {
            case NODE_STATUS_RX_IHU:
                /*
                 Here be dragons... This part of the code is responsible for causing lots of headache
                 We look at the bottom half of 3rd byte of '6A1' frame to determine what the "reply" should be
                 */
                switch (CAN_RxMsg.data[3] & 0x0F){
                    case (0x3):
//                        messageSender.sendCanMessage(NODE_STATUS_TX_CDC,cdcPoweronCmd,4,NODE_STATUS_TX_INTERVAL); // <-------------------
                        break;
                    case (0x2):
//                        messageSender.sendCanMessage(NODE_STATUS_TX_CDC,cdcActiveCmd,4,NODE_STATUS_TX_INTERVAL); // <--------------------
                        break;
                    case (0x8):
//                        messageSender.sendCanMessage(NODE_STATUS_TX_CDC,cdcPowerdownCmd,4,NODE_STATUS_TX_INTERVAL); // <-----------------
                        break;
                }
                break;
            case CDC_CONTROL:
                CDC_handleIhuButtons();
                break;
            case STEERING_WHEEL_BUTTONS:
                CDC_handleSteeringWheelButtons();
                break;
            case DISPLAY_RESOURCE_GRANT:
                if ((cdcActive) && (CAN_RxMsg.data[0] == 0x02)) {
                    if (CAN_RxMsg.data[1] == NODE_SID_FUNCTION_ID) {
                        // We have been granted the right to write text to the second row on the SID"
                        if (!writeTextOnDisplayTimerActive) {
//                            writeTextOnDisplayTimerId = time.every(SID_CONTROL_TX_BASETIME, &writeTextOnDisplayOnTime,NULL); // <--------
//                            writeTextOnDisplayTimerActive = true; // <-------------------------------------------------------------------
                        }
                    }
                    else {
                        // ”OK to write” = false
                    }
                }
                break;
            default:
                break;
                
        }
    }
}

/**
 * Handles the CDC_CONTROL frame that the IHU sends us when it wants to control some feature of the CDC
 */

void CDC_handleIhuButtons() {
    bool event = (CAN_RxMsg.data[0] == 0x80);
    if ((!event) && (cdcActive)) {
        CDC_checkCanEvent(1);
        return;
    }
    switch (CAN_RxMsg.data[1]) {
        case 0x24: // CDC = ON (CD/RDM button has been pressed twice)
            cdcActive = true;
#if (DEBUGMODE==1)
	    printf("cdcActive");
#endif
	startBluetooth();
//            BT.bt_reconnect(); // <---------------------------------------------------------------------------------------------------
            //sidWriteAccessWanted = true;
            //displayRequestTimerId = time.every(SID_CONTROL_TX_BASETIME, &sendDisplayRequestOnTime,NULL);
            CDC_sendCanFrame(SOUND_REQUEST, soundCmd);
            break;
        case 0x14: // CDC = OFF (Back to Radio or Tape mode)
            //sidWriteAccessWanted = false;
            //time.stop(writeTextOnDisplayTimerId);
            //writeTextOnDisplayTimerActive = false;
            //time.stop(displayRequestTimerId);
#if (DEBUGMODE==1)
	    printf("cdcOff");
#endif
	//restart??
	    esp_sleep_enable_timer_wakeup(1000);
	    esp_deep_sleep_start();

//            BT.bt_disconnect(); // <--------------------------------------------------------------------------------------------------
            cdcActive = false;
            break;
        default:
            break;
    }
    if ((event) && (CAN_RxMsg.data[1] != 0x00)) {
        if (cdcActive) {
            switch (CAN_RxMsg.data[1]) {
                case 0x59: // NXT
#if (DEBUGMODE==1)
		    printf("nxt");
#endif
//                    BT.bt_play(); // <------------------------------------------------------------------------------------------------
                    break;
                case 0x84: // SEEK button (middle) long press on IHU
#if (DEBUGMODE==1)
		    printf("seek - short");
#endif
                    break;
                case 0x88: // > 2 sec long press of SEEK button (middle) on IHU
#if (DEBUGMODE==1)
		    printf("seek - long");
#endif
                    break;
                case 0x76: // Random ON/OFF (Long press of CD/RDM button)
#if (DEBUGMODE==1)
		    printf("cd - long");
#endif
                    break;
                case 0xB1: // Pause ON
#if (DEBUGMODE==1)
		    printf("pause on");
#endif
//                    BT.bt_play(); // <------------------------------------------------------------------------------------------------
                    break;
                case 0xB0: // Pause OFF
#if (DEBUGMODE==1)
		    printf("pause off");
#endif
//                    BT.bt_play(); // <------------------------------------------------------------------------------------------------
                    break;
                case 0x35: // Track +
#if (DEBUGMODE==1)
		    printf("track+");
#endif
//                    BT.bt_next(); // <------------------------------------------------------------------------------------------------
                    break;
                case 0x36: // Track -
#if (DEBUGMODE==1)
		    printf("track-");
#endif
//                    BT.bt_prev(); // <------------------------------------------------------------------------------------------------
                    break;
                case 0x68: // IHU buttons "1-6"
                    switch (CAN_RxMsg.data[2]) {
                        case 0x01:
#if (DEBUGMODE==1)
			    printf("vol+");
#endif
//                            BT.bt_volup(); // <---------------------------------------------------------------------------------------
                            break;
                        case 0x02:
#if (DEBUGMODE==1)
			    printf("vol max");
#endif
//                            BT.bt_set_maxvol(); // <----------------------------------------------------------------------------------
                            break;
                        case 0x03:
#if (DEBUGMODE==1)
			    printf("reconnect");
#endif
//                            BT.bt_reconnect(); // <-----------------------------------------------------------------------------------
                            break;
                        case 0x04:
#if (DEBUGMODE==1)
			    printf("vol-");
#endif
//                            BT.bt_voldown(); // <-------------------------------------------------------------------------------------
                            break;
                        case 0x06:
#if (DEBUGMODE==1)
			    printf("disconnect");
#endif
//                            BT.bt_disconnect(); // <----------------------------------------------------------------------------------
                        default:
                            break;
                    }
                default:
                    break;
            }
        }
        cdcStatusResendNeeded = true;
        cdcStatusResendDueToCdcCommand = true;
    }
}

/**
 * Handles the STEERING_WHEEL_BUTTONS frame
 * TODO connect the SID button events to actions
 */

void CDC_handleSteeringWheelButtons() {
    if (cdcActive) {
        CDC_checkCanEvent(4);
        switch (CAN_RxMsg.data[2]) {
            case 0x04: // NXT button on wheel
#if (DEBUGMODE==1)
		    printf("nxt");
#endif
                //BT.bt_play();
                break;
            case 0x10: // Seek+ button on wheel
#if (DEBUGMODE==1)
		    printf("seek+");
#endif
                //BT.bt_next();
                break;
            case 0x08: // Seek- button on wheel
#if (DEBUGMODE==1)
		    printf("seek-");
#endif
                //BT.bt_prev();
                break;
            default:
                break;
        }
    }
}

/**
 * Handles CDC status and sends it to IHU as necessary
 */

void CDC_handleCdcStatus() {
    
    CDC_handleRxFrame();
    
    // If the CDC status frame needs to be sent as an event, do so now
    // (note though, that we may not send the frame more often than once every 50 ms)
    
    if (cdcStatusResendNeeded && (millis() - cdcStatusLastSendTime > 50)) {
        CDC_sendCdcStatus(cdcStatusResendNeeded, cdcStatusResendDueToCdcCommand, cdcActive);
    }
    
    // CDC status frame must be sent with a 1000 ms periodicity:
    if (millis() - cdcStatusLastSendTime > CDC_STATUS_TX_BASETIME) {
        // Send the CDC status frame, marked periodical and triggered internally:
        CDC_sendCdcStatus(cdcStatusResendNeeded, cdcStatusResendDueToCdcCommand, cdcActive);
    }
}

void CDC_sendCdcStatus(bool event, bool remote, bool cdcActive) {
    
    /* Format of GENERAL_STATUS_CDC frame:
     ID: CDC node ID
     [0]:
     byte 0, bit 7: FCI NEW DATA: 0 - sent on basetime, 1 - sent on event
     byte 0, bit 6: FCI REMOTE CMD: 0 - status change due to internal operation, 1 - status change due to CDC_COMMAND frame
     byte 0, bit 5: FCI DISC PRESENCE VALID: 0 - disc presence signal is not valid, 1 - disc presence signal is valid
     [1]: Disc presence validation (boolean)
     byte 1-2, bits 0-15: DISC PRESENCE: (bitmap) 0 - disc absent, 1 - disc present. Bit 0 is disc 1, bit 1 is disc 2, etc.
     [2]: Disc presence (bitmap)
     byte 1-2, bits 0-15: DISC PRESENCE: (bitmap) 0 - disc absent, 1 - disc present. Bit 0 is disc 1, bit 1 is disc 2, etc.
     [3]: Disc number currently playing
     byte 3, bits 7-4: DISC MODE
     byte 3, bits 3-0: DISC NUMBER
     [4]: Track number currently playing
     [5]: Minute of the current track
     [6]: Second of the current track
     [7]: CD changer status; D0 = Married to the car
     */
    
    uint8_t discMode          = 0x05;  // Play; 0x0E can also be tried for "test mode" but might stop IHU from updating the display
    
    unsigned char cdcGeneralStatusCmd[CAN_FRAME_LENGTH];
    cdcGeneralStatusCmd[0] = ((event ? 0x07 : 0x00) | (remote ? 0x00 : 0x01)) << 5;
    cdcGeneralStatusCmd[1] = (cdcActive ? 0xFF : 0x00);                             // Validation for presence of six discs in the magazine
    cdcGeneralStatusCmd[2] = (cdcActive ? 0x3F : 0x01);                             // There are six discs in the magazine
    cdcGeneralStatusCmd[3] = (cdcActive ? 0x41 : 0x01);                             // ToDo: check 0x01 | (discMode << 4) | 0x01
    cdcGeneralStatusCmd[4] = 0xFF;
    cdcGeneralStatusCmd[5] = 0xFF;
    cdcGeneralStatusCmd[6] = 0xFF;
    cdcGeneralStatusCmd[7] = 0xD0;

    CDC_sendCanFrame(GENERAL_STATUS_CDC, cdcGeneralStatusCmd);

    // Record the time of sending and reset status variables
    cdcStatusLastSendTime = millis();
    cdcStatusResendNeeded = false;
    cdcStatusResendDueToCdcCommand = false;
    
}

/**
 * Sends a request for using the SID, row 2. We may NOT start writing until we've received a grant frame with the correct function ID!
 */

void CDC_sendDisplayRequest(bool sidWriteAccessWanted) {
    
    /* Format of NODE_DISPLAY_RESOURCE_REQ frame:
     ID: Node ID requesting to write on SID
     [0]: Request source
     [1]: SID object to write on; 0 = entire SID; 1 = 1st row; 2 = 2nd row
     [2]: Request type: 1 = Engineering test; 2 = Emergency; 3 = Driver action; 4 = ECU action; 5 = Static text; 0xFF = We don't want to write on SID
     [3]: Request source function ID
     [4-7]: Zeroed out; not in use
     */
    
    unsigned char displayRequestCmd[CAN_FRAME_LENGTH];
    displayRequestCmd[0] = NODE_APL_ADR;
    displayRequestCmd[1] = 0x02;
    displayRequestCmd[2] = (sidWriteAccessWanted ? 0x05 : 0xFF);
    displayRequestCmd[3] = NODE_SID_FUNCTION_ID;
    displayRequestCmd[4] = 0x00;
    displayRequestCmd[5] = 0x00;
    displayRequestCmd[6] = 0x00;
    displayRequestCmd[7] = 0x00;

    CDC_sendCanFrame(NODE_DISPLAY_RESOURCE_REQ, displayRequestCmd);
}

/**
 * Formats and puts a frame on CAN bus
 */

void CDC_sendCanFrame(int messageId, unsigned char *msg) {
    CAN_TxMsg.id = messageId;
    for (int i = 0; i < CAN_FRAME_LENGTH; i++) {
        CAN_TxMsg.data[i] = msg[i];
    }
    CAN_send(&CAN_TxMsg);
}

/**
 * Sends display request every SID_CONTROL_TX_BASETIME interval
 */

void sendDisplayRequestOnTime(/* void* */) {
    CDC_sendDisplayRequest(sidWriteAccessWanted);
}

/**
 * Writes provided text every SID_CONTROL_TX_BASETIME interval
 */

void writeTextOnDisplayOnTime(/* void* */) {
    CDC_writeTextOnDisplay(MODULE_NAME);
}

/**
 * Formats provided text for writing on the SID. This function assumes that we have been granted write access. Do not call it if we haven't!
 * Note: the character set used by the SID is slightly nonstandard. "Normal" characters should work fine.
 */

void CDC_writeTextOnDisplay(const char textIn[]) {

    if (!textIn) {
        return;
    }
    // Copy the provided string and make sure we have a new array of the correct length
    char textToSid[15];
    int i, m, n;
    n = strlen(textIn);
    n = n > 12 ? 12 : n;      // 12 is the number of characters SID can display on each row; anything beyond 12 is going to be zeroed out
    for (i = 0; i < n; i++) {
        textToSid[i] = textIn[i];
    }
    for (i = n; i < 15; i++) {
        textToSid[i] = 0;
    }
    
    unsigned char sidMessageGroup[3][CAN_FRAME_LENGTH] = {
        {0x42,0x96,0x02,textToSid[0],textToSid[1],textToSid[2],textToSid[3],textToSid[4]},
        {0x01,0x96,0x02,textToSid[5],textToSid[6],textToSid[7],textToSid[8],textToSid[9]},
        {0x00,0x96,0x02,textToSid[10],textToSid[11],textToSid[12],textToSid[13],textToSid[14]}
    };
    
    // messageSender.sendCanMessage(NODE_WRITE_TEXT_ON_DISPLAY,sidMessageGroup,3,10); <-----------------------------------------------!!!!

}

/**
 * Checks for a long press of a button event
 * A long press is considered if the first byte of CAN frame != 0x80
 * Then we look at 'frameElement' passed in by function call to look at which element we should put the counter on
 * Once we reach 3 on the counter, we assert that a certain button has been held for a while and take an action accordingly
 * LAST_EVENT_IN_TIMEOUT indicates how many milliseconds have to pass till we reset all the counters and wait for the next potential long press to come in
 */

void CDC_checkCanEvent(int frameElement) {
    bool event = (CAN_RxMsg.data[0] == 0x80);
    if (!event && (CAN_RxMsg.data[frameElement]) != 0) { // Long press of a steering wheel button has taken place.
        if (millis() - lastIcomingEventTime > LAST_EVENT_IN_TIMEOUT) {
            incomingEventCounter = 0;
        }
        incomingEventCounter++;
        lastIcomingEventTime = millis();
        if (incomingEventCounter == 3) {
            switch (CAN_RxMsg.data[frameElement]) {
                case 0x04: // NXT button on steering wheel
#if (DEBUGMODE==1)
		    printf("nxt - long?");
#endif
//                    BT.bt_vassistant(); // <------------------------------------------------------------------------------------
                    break;
                case 0x45: // SEEK+ button long press on IHU
#if (DEBUGMODE==1)
		    printf("seek+ - long?");
#endif
//                    BT.bt_visible(); // <---------------------------------------------------------------------------------------
                    CDC_sendCanFrame(SOUND_REQUEST, soundCmd);
                    break;
                case 0x46: // SEEK- button long press on IHU
#if (DEBUGMODE==1)
		    printf("seek- - long?");
#endif
//                    BT.bt_reboot(); // <----------------------------------------------------------------------------------------
                    CDC_sendCanFrame(SOUND_REQUEST, soundCmd);
                    break;
                case 0x68: // IHU buttons "1-6"
#if (DEBUGMODE==1)
		    printf("ihu numbers");
#endif
                    switch (CAN_RxMsg.data[2]) {
                        case 0x03:
//                            BT.bt_visible(); // <------------------------------------------------------------------------------
                            CDC_sendCanFrame(SOUND_REQUEST, soundCmd);
                            break;
                        case 0x06:
//                            BT.bt_reboot(); // <-------------------------------------------------------------------------------
                            CDC_sendCanFrame(SOUND_REQUEST, soundCmd);
                            break;
                        default:
                            break;
                    }
                default:
                    break;
            }
        }
    }
    return;
}
