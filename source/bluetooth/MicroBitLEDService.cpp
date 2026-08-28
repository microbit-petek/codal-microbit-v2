/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

/**
  * Class definition for the custom MicroBit LED Service.
  * Provides a BLE service to remotely read and write the state of the LED display.
  */
#include "CodalFiber.h"
#include "EventModel.h"
#include "MicroBitBLEService.h"
#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitLEDService.h"

using namespace codal;

const uint16_t MicroBitLEDService::serviceUUID               = 0xd91d;
const uint16_t MicroBitLEDService::charUUID[ mbbs_cIdxCOUNT] = { 0x7b77, 0x93ee, 0x0d2d };

/**
  * Constructor.
  * Create a representation of the LEDService
  * @param _ble The instance of a BLE device that we're running on.
  * @param _display An instance of MicroBitDisplay to interface with.
  */
MicroBitLEDService::MicroBitLEDService( BLEDevice &_ble, MicroBitDisplay &_display, UartBle &uartBle) :
    MicroBitBLEService(uartBle),
    display(_display)
{
    // Initialise our characteristic values.
    memclr( matrixValue, sizeof( matrixValue));
    textValue[0]    = 0;
    speedValue      = MICROBIT_DEFAULT_SCROLL_SPEED;
    
    // Register the base UUID and create the service.
    RegisterBaseUUID( bs_base_uuid);
    CreateService( serviceUUID);
    
    // Add each of our characteristics.
    CreateCharacteristic( mbbs_cIdxMATRIX, charUUID[ mbbs_cIdxMATRIX],
                         (uint8_t *)matrixValue,
                         sizeof(matrixValue), sizeof(matrixValue),
                         microbit_propWRITE | microbit_propREAD | microbit_propREADAUTH);
    
    CreateCharacteristic( mbbs_cIdxTEXT,   charUUID[ mbbs_cIdxTEXT],
                         (uint8_t *)textValue,
                         sizeof(textValue), sizeof(textValue),
                         microbit_propWRITE);
    
    CreateCharacteristic( mbbs_cIdxSPEED,  charUUID[ mbbs_cIdxSPEED],
                         (uint8_t *)&speedValue,
                         sizeof(speedValue), sizeof(speedValue),
                         microbit_propWRITE | microbit_propREAD);

    EventModel::defaultEventBus->listen(UARTBLE_ID, LED_DATA_WRITE, this, &MicroBitLEDService::serialDataWrite);
    EventModel::defaultEventBus->listen(UARTBLE_ID, LED_DATA_REQUEST, this, &MicroBitLEDService::serialDataRequest);
    EventModel::defaultEventBus->listen(UARTBLE_ID, LED_TEXT_WRITE, this, &MicroBitLEDService::serialTextWrite);
    EventModel::defaultEventBus->listen(UARTBLE_ID, LED_SCROLLING_DELAY_WRITE, this, &MicroBitLEDService::serialScrollingDelayWrite);
}


/**
  * Callback. Invoked when any of our attributes are written via BLE.
  */
void MicroBitLEDService::onDataWritten( const microbit_ble_evt_write_t *params)
{
    if (params->handle == valueHandle( mbbs_cIdxMATRIX) && params->len > 0 && params->len < 6)
    {
        uartBle.sendMessage(LED_DATA_WRITE, params->data, params->len);
    }

    else if (params->handle == valueHandle( mbbs_cIdxTEXT))
    {
        uint8_t const payloadLength = params->len + 1;
        uint8_t * const payload = new uint8_t[payloadLength];
        payload[0] = params->len;
        memcpy(&payload[1], params->data, params->len);
        uartBle.sendMessage(LED_TEXT_WRITE, payload, payloadLength);
    }

    else if (params->handle == valueHandle( mbbs_cIdxSPEED) && params->len >= sizeof(speedValue))
    {
        uartBle.sendMessage(LED_SCROLLING_DELAY_WRITE, params->data, params->len);
    }
}

void MicroBitLEDService::serialDataWrite(Event)
{
    memcpy(matrixValue, uartBle.ledData, sizeof(matrixValue));

   // interrupt any animation that might be currently going on
   display.stopAnimation();
   for (uint8_t y=0; y<sizeof(matrixValue); y++)
        for (int x=0; x<5; x++)
            display.image.setPixelValue(x, y, (matrixValue[y] & (0x01 << (4-x))) ? 255 : 0);
}

void MicroBitLEDService::serialTextWrite(Event)
{
    // Create a ManagedString representation from the UTF8 data.
    // We do this explicitly to control the length (in case the string is not NULL terminated!)
    ManagedString s((char *)uartBle.ledRawText->data, uartBle.ledRawText->length);

    // interrupt any animation that might be currently going on
    display.stopAnimation();

    // Start the string scrolling and we're done.
    display.scrollAsync(s, (int) speedValue);
}

void MicroBitLEDService::serialScrollingDelayWrite(Event)
{
    speedValue = uartBle.ledScrollDelay;
}

/**
  * Callback. Invoked when any of our attributes are read via BLE.
  * Set  params->data and params->length to update the value
  */
void MicroBitLEDService::onDataRead( microbit_onDataRead_t *params)
{
    if ( params->handle == valueHandle( mbbs_cIdxMATRIX))
    {
        uartBle.sendMessage(LED_DATA_REQUEST, NULL, 0);

        fiber_wait_for_event(UARTBLE_ID, LED_DATA_UPDATE);

        memcpy(matrixValue, uartBle.ledData, sizeof(matrixValue));
    }
}

void MicroBitLEDService::serialDataRequest(Event)
{
    for (int y=0; y<5; y++)
    {
        matrixValue[y] = 0;

        for (int x=0; x<5; x++)
        {
            if (display.image.getPixelValue(x, y))
                matrixValue[y] |= 0x01 << (4-x);
        }
    }

    uartBle.sendMessage(LED_DATA_UPDATE, matrixValue, sizeof(matrixValue));
}

#endif
