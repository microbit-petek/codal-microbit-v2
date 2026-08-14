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
        case UARTBLEMESSAGEID_MAX:
        {
            continue;
        }
        }
    }
}

void UartBle::handleAccelerometerDataUpdate()
{
    serial.read((uint8_t *)&accelerometerData[0], sizeof(accelerometerData[0]));
    serial.read((uint8_t *)&accelerometerData[1], sizeof(accelerometerData[1]));
    serial.read((uint8_t *)&accelerometerData[2], sizeof(accelerometerData[2]));
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
    accelerometerData[0] = accelerometer.getX();
    accelerometerData[1] = accelerometer.getY();
    accelerometerData[2] = accelerometer.getZ();
    accelerometerPeriodMs = accelerometer.getPeriod();
    EventModel::defaultEventBus->listen(MICROBIT_ID_ACCELEROMETER,
                                        MICROBIT_ACCELEROMETER_EVT_DATA_UPDATE, this,
                                        &UartBle::accelerometerDataHandler);
    sendMessage(ACCELEROMETER_DATA_UPDATE, accelerometerData, sizeof(accelerometerData));
    sendMessage(ACCELEROMETER_PERIOD_UPDATE, &accelerometerPeriodMs, sizeof(accelerometerPeriodMs));
}

void UartBle::handleBleDisconnected()
{
    EventModel::defaultEventBus->ignore(MICROBIT_ID_ACCELEROMETER,
                                        MICROBIT_ACCELEROMETER_EVT_DATA_UPDATE, this,
                                        &UartBle::accelerometerDataHandler);
}
