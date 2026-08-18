#include "UARTBLE.h"
#include "MicroBitCompat.h"

using namespace codal;

UartBle::UartBle(NRF52Serial &serial) : serial(serial) {}

void UartBle::sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength)
{
    serial.sendChar(id);
    if ((payload != NULL) && (payloadLength > 0))
    {
        serial.send((uint8_t *)payload, payloadLength);
    }
}

void UartBle::runRx()
{

    while (true)
    {
        uint8_t rawId = serial.read();

        if (rawId >= UARTBLEMESSAGEID_MAX)
        {
            continue;
        }

        switch ((UartBleMessageId)rawId)
        {
        case ACCELEROMETER_DATA_UPDATE:
        {
            serial.read((uint8_t *)&accelerometerData, sizeof(accelerometerData));
            break;
        }
        case ACCELEROMETER_PERIOD_WRITE:
        case ACCELEROMETER_PERIOD_UPDATE:
        {
            serial.read((uint8_t *)&accelerometerPeriodMs, sizeof(accelerometerPeriodMs));
            break;
        }
        case MAGNETOMETER_DATA_UPDATE:
        {
            serial.read((uint8_t *)&magnetometerData, sizeof(magnetometerData));
            break;
        }
        case MAGNETOMETER_BEARING_UPDATE:
        {
            serial.read((uint8_t *)&magnetometerBearing, sizeof(magnetometerBearing));
            break;
        }
        case MAGNETOMETER_PERIOD_WRITE:
        case MAGNETOMETER_PERIOD_UPDATE:
        {
            serial.read((uint8_t *)&magnetometerPeriodMs, sizeof(magnetometerPeriodMs));
            break;
        }
        case MAGNETOMETER_CALIBRATION_UPDATE:
        {
            magnetometerCalibration = serial.read();
            break;
        }
        case MAGNETOMETER_CALIBRATION_REQUESTED:
        {
            break;
        }
        case BLE_CONNECTED:
        case BLE_DISCONNECTED:
        {
            break;
        }
        case UARTBLEMESSAGEID_MAX:
        case ID_RESERVED:
        {
            continue;
        }
        }

        Event(MICROBIT_ID_SERIAL, rawId);
    }
}
