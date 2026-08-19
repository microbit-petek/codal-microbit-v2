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
 * Class definition for the custom MicroBit IOPin Service.
 * Provides a BLE service to remotely read the state of the I/O Pin, and configure its behaviour.
 */
#include "MicroBitBLEService.h"
#include "MicroBitConfig.h"

#if CONFIG_ENABLED(DEVICE_BLE)

#include "MicroBitIOPinService.h"
#include "MicroBitFiber.h"

using namespace codal;

const uint16_t MicroBitIOPinService::serviceUUID = 0x127b;
const uint16_t MicroBitIOPinService::charUUID[mbbs_cIdxCOUNT] = {0x5899, 0xb9fe, 0xd822, 0x8d00};

/**
 * Constructor.
 * Create a representation of the IOPinService
 * @param _ble The instance of a BLE device that we're running on.
 * @param _io An instance of MicroBitIO that this service will use to perform
 *            I/O operations.
 */
MicroBitIOPinService::MicroBitIOPinService(BLEDevice &_ble, MicroBitIO &_io, UartBle &uartBle)
    : MicroBitBLEService(uartBle), io(_io)
{
    // Initialise our characteristic values.
    ioPinServiceADCharacteristicBuffer = 0;
    ioPinServiceIOCharacteristicBuffer = 0;
    ioPinServiceADSetting = 0;
    ioPinServiceIOSetting = 0;
    memset(ioPinServiceIOData, 0, sizeof(ioPinServiceIOData));
    memset(ioPinServicePWMCharacteristicBuffer, 0,
           sizeof(ioPinServicePWMCharacteristicBuffer)); // Create the AD characteristic, that
                                                         // defines whether each pin is treated as
                                                         // analogue or digital

    // Register the base UUID and create the service.
    RegisterBaseUUID(bs_base_uuid);
    CreateService(serviceUUID);

    CreateCharacteristic(
        mbbs_cIdxADC, charUUID[mbbs_cIdxADC], (uint8_t *)&ioPinServiceADCharacteristicBuffer,
        sizeof(ioPinServiceADCharacteristicBuffer), sizeof(ioPinServiceADCharacteristicBuffer),
        microbit_propREAD | microbit_propWRITE);

    // Create the IO characteristic, that allows the user to define one or more pins as inputs.
    // These will then be monitored by this service and reported via the ioPinSeriveData
    // characterisitic.
    CreateCharacteristic(
        mbbs_cIdxIO, charUUID[mbbs_cIdxIO], (uint8_t *)&ioPinServiceIOCharacteristicBuffer,
        sizeof(ioPinServiceIOCharacteristicBuffer), sizeof(ioPinServiceIOCharacteristicBuffer),
        microbit_propREAD | microbit_propWRITE);

    // Create the PWM characteristic, that allows up to 3 compatible pins to be used for PWM
    CreateCharacteristic(mbbs_cIdxPWM, charUUID[mbbs_cIdxPWM],
                         (uint8_t *)&ioPinServicePWMCharacteristicBuffer,
                         sizeof(ioPinServicePWMCharacteristicBuffer),
                         sizeof(ioPinServicePWMCharacteristicBuffer), microbit_propWRITE);

    // Create the Data characteristic, that allows the actual read and write operations.
    CreateCharacteristic(
        mbbs_cIdxDATA, charUUID[mbbs_cIdxDATA], (uint8_t *)ioPinServiceDataCharacteristicBuffer, 0,
        sizeof(ioPinServiceDataCharacteristicBuffer),
        microbit_propREAD | microbit_propWRITE | microbit_propNOTIFY | microbit_propREADAUTH);

    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_DATA_WRITE, this,
                                        &MicroBitIOPinService::serialDataWrite);
    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_DATA_UPDATE, this,
                                        &MicroBitIOPinService::serialDataUpdate);
    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_DATA_REQUEST, this,
                                        &MicroBitIOPinService::serialDataRequest);
    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_AD_CONFIGURATION_WRITE, this,
                                        &MicroBitIOPinService::serialADConfigurationWrite);
    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_IO_CONFIGURATION_WRITE, this,
                                        &MicroBitIOPinService::serialIOConfigurationWrite);
    EventModel::defaultEventBus->listen(UARTBLE_ID, PIN_PWM_WRITE, this,
                                        &MicroBitIOPinService::serialPwmWrite);

    fiber_add_idle_component(this);
}

MicroBitPin &MicroBitIOPinService::edgePin(int index)
{
    if (index < 0 || index >= MICROBIT_IO_PIN_SERVICE_PINCOUNT)
        index = 0;
    return io.pin[index];
};

/**
 * Determines if the given pin was configured as a digital pin by the BLE
 * ADPinConfigurationCharacterisitic.
 *
 * @param i the enumeration of the pin to test
 * @return 1 if this pin is configured as digital, 0 otherwise
 */
int MicroBitIOPinService::isDigital(int i)
{
    return ((ioPinServiceADSetting & (1 << i)) == 0);
}

/**
 * Determines if the given pin was configured as an analog pin by the BLE
 * ADPinConfigurationCharacterisitic.
 *
 * @param i the enumeration of the pin to test
 * @return 1 if this pin is configured as analog, 0 otherwise
 */
int MicroBitIOPinService::isAnalog(int i)
{
    return ((ioPinServiceADSetting & (1 << i)) != 0);
}

/**
 * Determines if the given pin was configured as an input by the BLE
 * IOPinConfigurationCharacterisitic.
 *
 * @param i the enumeration of the pin to test
 * @return 1 if this pin is configured as an input, 0 otherwise
 */
int MicroBitIOPinService::isActiveInput(int i)
{
    return ((ioPinServiceIOSetting & (1 << i)) != 0);
}

/**
 * Scans through all pins that our BLE client have registered an interest in.
 * For each pin that has changed value, update the BLE characteristic, and NOTIFY our client.
 */
int MicroBitIOPinService::updateBLEInputs(bool updateAll)
{
    int pairs = 0;

    for (int i = 0; i < MICROBIT_IO_PIN_SERVICE_PINCOUNT; i++)
    {
        if (isActiveInput(i))
        {
            uint8_t value;

            if (isDigital(i))
                value = edgePin(i).getDigitalValue();
            else
                value = edgePin(i).getAnalogValue() >> 2;

            // If the data has changed, send an update.
            if (updateAll || value != ioPinServiceIOData[i])
            {
                ioPinServiceIOData[i] = value;

                ioPinServiceDataCharacteristicBuffer[pairs].pin = i;
                ioPinServiceDataCharacteristicBuffer[pairs].value = value;

                pairs++;

                if (pairs >= MICROBIT_IO_PIN_SERVICE_DATA_SIZE)
                    break;
            }
        }
    }

    return pairs;
}

/**
 * Callback. Invoked when any of our attributes are written via BLE.
 */
void MicroBitIOPinService::onDataWritten(const microbit_ble_evt_write_t *params)
{
    // Check for writes to the IO configuration characteristic
    if (params->handle == valueHandle(mbbs_cIdxIO) &&
        params->len >= sizeof(ioPinServiceIOCharacteristicBuffer))
    {
        uartBle.sendMessage(PIN_IO_CONFIGURATION_WRITE, &ioPinServiceIOCharacteristicBuffer,
                            sizeof(ioPinServiceIOCharacteristicBuffer));
    }

    // Check for writes to the IO configuration characteristic
    if (params->handle == valueHandle(mbbs_cIdxADC) &&
        params->len >= sizeof(ioPinServiceADCharacteristicBuffer))
    {
        uartBle.sendMessage(PIN_AD_CONFIGURATION_WRITE, &ioPinServiceADCharacteristicBuffer,
                            sizeof(ioPinServiceADCharacteristicBuffer));
    }

    // Check for writes to the PWM Control characteristic
    if (params->handle == valueHandle(mbbs_cIdxPWM))
    {
        uint8_t const payloadLength = params->len + 1;
        uint8_t *const payloadBuffer = new uint8_t[payloadLength];
        payloadBuffer[0] = params->len;
        memcpy(&payloadBuffer[1], params->data, params->len);
        uartBle.sendMessage(PIN_PWM_WRITE, payloadBuffer, payloadLength);
        delete[] payloadBuffer;
    }

    if (params->handle == valueHandle(mbbs_cIdxDATA))
    {
        uint8_t const payloadLength = params->len + 1;
        uint8_t *const payloadBuffer = new uint8_t[payloadLength];
        payloadBuffer[0] = params->len;
        memcpy(&payloadBuffer[1], params->data, params->len);
        uartBle.sendMessage(PIN_DATA_WRITE, payloadBuffer, payloadLength);
        delete[] payloadBuffer;
    }
}

void MicroBitIOPinService::serialDataWrite(Event)
{
    // We have some pin data to change...
    uint8_t len = uartBle.pinData->length;
    IOData const *data = (IOData *)uartBle.pinData->data;

    // There may be multiple write operations... take each in turn and update the pin values
    while (len >= sizeof(IOData))
    {
        if (!isActiveInput(data->pin))
        {
            if (isDigital(data->pin))
                edgePin(data->pin).setDigitalValue(data->value);
            else
                edgePin(data->pin).setAnalogValue(data->value == 255 ? 1023 : data->value << 2);
        }

        data++;
        len -= sizeof(IOData);
    }
}

void MicroBitIOPinService::serialADConfigurationWrite(Event)
{
    ioPinServiceADSetting = uartBle.pinADConfiguration;

    // Also, drop any selected pins into input mode, so we can pick up changes later
    for (int i = 0; i < MICROBIT_IO_PIN_SERVICE_PINCOUNT; i++)
    {
        if (isDigital(i) && isActiveInput(i))
            edgePin(i).getDigitalValue();

        if (isAnalog(i) && isActiveInput(i))
            edgePin(i).getAnalogValue();
    }
}
void MicroBitIOPinService::serialIOConfigurationWrite(Event)
{
    ioPinServiceIOSetting = uartBle.pinIOConfiguration;

    // Also, drop any selected pins into input mode, so we can pick up changes later
    for (int i = 0; i < MICROBIT_IO_PIN_SERVICE_PINCOUNT; i++)
    {
        if (isDigital(i) && isActiveInput(i))
            edgePin(i).getDigitalValue();

        if (isAnalog(i) && isActiveInput(i))
            edgePin(i).getAnalogValue();
    }
}

void MicroBitIOPinService::serialPwmWrite(Event)
{
    uint8_t const len = uartBle.pinPwmControl->length;
    IOPWMData const *const pwmData = (IOPWMData *)uartBle.pinPwmControl->data;

    // validate - len must be a multiple of 7 and greater than 0
    bool is_valid_length = len > 0 && len % 7 == 0;
    if (is_valid_length)
    {
        uint8_t field_count = len / 7;
        for (int i = 0; i < field_count; i++)
        {
            uint8_t pin = pwmData[i].pin;
            uint16_t value = pwmData[i].value;
            uint32_t period = pwmData[i].period;
            edgePin(pin).setAnalogValue(value);
            edgePin(pin).setAnalogPeriodUs(period);
        }
    }
    else
    {
        // there's no way to return an error response via the current mbed BLE API :-( See
        // https://github.com/ARMmbed/ble/issues/181
        return;
    }
}

/**
 * Callback. invoked when the BLE data characteristic is read.
 * Set  params->data and params->length to update the value
 *
 * Reads all the pins marked as inputs, and updates the data stored in the characteristic.
 */
void MicroBitIOPinService::onDataRead(microbit_onDataRead_t *params)
{
    if (params->handle == valueHandle(mbbs_cIdxDATA))
    {
        uartBle.sendMessage(PIN_DATA_REQUEST, NULL, 0);

        fiber_wait_for_event(UARTBLE_ID, PIN_DATA_UPDATE);

        if (getConnected())
        {
            memcpy(ioPinServiceDataCharacteristicBuffer, uartBle.pinData->data,
                   uartBle.pinData->length);
            params->data = (uint8_t *)ioPinServiceDataCharacteristicBuffer;
            params->length = uartBle.pinData->length;
        }
    }
}

void MicroBitIOPinService::serialDataRequest(Event)
{
    int pairs = updateBLEInputs(true);
    uint8_t const dataLength = (sizeof(IOData) * pairs);
    uint8_t const payloadLength = dataLength + 1;
    uint8_t *const payloadBuffer = new uint8_t[payloadLength];
    payloadBuffer[0] = dataLength;
    memcpy(&payloadBuffer[1], ioPinServiceDataCharacteristicBuffer, dataLength);
    uartBle.sendMessage(PIN_DATA_UPDATE, payloadBuffer, payloadLength);
    delete[] payloadBuffer;
}

/**
 * Periodic callback from MicroBit scheduler.
 *
 * Check if any of the pins we're watching need updating. Notify any connected
 * device with any changes.
 */
void MicroBitIOPinService::idleCallback()
{
    int pairs = updateBLEInputs(false);
    // If there's any data, issue a BLE notification.
    if (pairs)
    {
        uint8_t const dataLength = (sizeof(IOData) * pairs);
        uint8_t const payloadLength = dataLength + 1;
        uint8_t *const payloadBuffer = new uint8_t[payloadLength];
        payloadBuffer[0] = dataLength;
        memcpy(&payloadBuffer[1], ioPinServiceDataCharacteristicBuffer, dataLength);
        uartBle.sendMessage(PIN_DATA_UPDATE, payloadBuffer, payloadLength);
        delete[] payloadBuffer;
    }
}

void MicroBitIOPinService::serialDataUpdate(Event)
{
    if (getConnected() && (uartBle.pinData->length > 0))
    {
        memcpy(ioPinServiceDataCharacteristicBuffer, uartBle.pinData->data,
               uartBle.pinData->length);
        notifyChrValue(mbbs_cIdxDATA, (uint8_t *)ioPinServiceDataCharacteristicBuffer,
                       uartBle.pinData->length);
    }
}

#endif
