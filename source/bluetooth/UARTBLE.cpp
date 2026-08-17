#include "UARTBLE.h"
#include "EventModel.h"

using namespace codal;

UartBle::UartBle(NRF52Serial &serial, Accelerometer &accelerometer)
    : serial(serial), accelerometer(accelerometer)
{
}

void UartBle::sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength)
{
    serial.sendChar(id);
    if ((payload != NULL) && (payloadLength > 0))
    {
        serial.send((uint8_t *)payload, payloadLength);
    }
}

void UartBle::accelerometerDataHandler(MicroBitEvent const)
{
    accelerometerData[0] = accelerometer.getX();
    accelerometerData[1] = accelerometer.getY();
    accelerometerData[2] = accelerometer.getZ();
    sendMessage(ACCELEROMETER_DATA_UPDATE, accelerometerData, sizeof(accelerometerData));
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
            handleAccelerometerDataUpdate();
            break;
        }
        case ACCELEROMETER_PERIOD_WRITE:
        {
            handleAccelerometerPeriodWrite();
            break;
        }
        case ACCELEROMETER_PERIOD_UPDATE:
        {
            handleAccelerometerPeriodUpdate();
            break;
        }
        case BLE_CONNECTED:
        {
            handleBleConnected();
            break;
        }
        case BLE_DISCONNECTED:
        {
            handleBleDisconnected();
            break;
        }
        case MAGNETOMETER_DATA_UPDATE:
        {
            handleMagnetometerDataUpdate();
            break;
        }
        case MAGNETOMETER_BEARING_UPDATE:
        {
            handleMagnetometerBearingUpdate();
            break;
        }
        case MAGNETOMETER_PERIOD_WRITE:
        {
            handleMagnetometerPeriodWrite();
            break;
        }
        case MAGNETOMETER_PERIOD_UPDATE:
        {
            handleMagnetometerPeriodUpdate();
            break;
        }
        case MAGNETOMETER_CALIBRATION_UPDATE:
            {
                handleMagnetometerCalibrationUpdate();
                break;
            }
        case MAGNETOMETER_CALIBRATION_REQUESTED:
            {
                handleMagnetometerCalibrationRequested();
                break;
            }
        case UARTBLEMESSAGEID_MAX:
        case ID_RESERVED:
        {
            continue;
        }
        }
    }
}

void UartBle::handleAccelerometerDataUpdate()
{
    serial.read((uint8_t *)&accelerometerData, sizeof(accelerometerData));
    Event(MICROBIT_ID_SERIAL, ACCELEROMETER_DATA_UPDATE);
}

void UartBle::handleAccelerometerPeriodWrite()
{
    serial.read((uint8_t *)&accelerometerPeriodMs, sizeof(accelerometerPeriodMs));
    accelerometer.setPeriod(accelerometerPeriodMs);

    // Read back the actual period set by the accelerometer
    accelerometerPeriodMs = accelerometer.getPeriod();
    sendMessage(ACCELEROMETER_PERIOD_UPDATE, &accelerometerPeriodMs, sizeof(accelerometerPeriodMs));
}

void UartBle::handleAccelerometerPeriodUpdate()
{
    serial.read((uint8_t *)&accelerometerPeriodMs, sizeof(accelerometerPeriodMs));
    Event(MICROBIT_ID_SERIAL, ACCELEROMETER_PERIOD_UPDATE);
}

void UartBle::handleBleConnected()
{
    Event(MICROBIT_ID_SERIAL, BLE_CONNECTED);
}

void UartBle::handleBleDisconnected()
{
    Event(MICROBIT_ID_SERIAL, BLE_DISCONNECTED);
}

void UartBle::handleMagnetometerDataUpdate()
{
    serial.read((uint8_t *)&magnetometerData, sizeof(magnetometerData));
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_DATA_UPDATE);
}

void UartBle::handleMagnetometerBearingUpdate()
{
    serial.read((uint8_t *)&magnetometerBearing, sizeof(magnetometerBearing));
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_BEARING_UPDATE);
}

void UartBle::handleMagnetometerPeriodWrite()
{
    serial.read((uint8_t *)&magnetometerPeriodMs, sizeof(magnetometerPeriodMs));
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_PERIOD_WRITE);
}

void UartBle::handleMagnetometerPeriodUpdate()
{
    serial.read((uint8_t *)&magnetometerPeriodMs, sizeof(magnetometerPeriodMs));
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_PERIOD_UPDATE);
}

void UartBle::handleMagnetometerCalibrationUpdate()
{
    magnetometerCalibration = serial.read();
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_CALIBRATION_UPDATE);
}

void UartBle::handleMagnetometerCalibrationRequested()
{
    Event(MICROBIT_ID_SERIAL, MAGNETOMETER_CALIBRATION_REQUESTED);
}
