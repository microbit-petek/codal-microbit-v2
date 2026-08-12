#include "MicroBitAccelerometer.h"
#include "NRF52Serial.h"

enum UartBleMessageId
{
    BLE_CONNECTED,
    BLE_DISCONNECTED,
    ACCELEROMETER_DATA_UPDATE,
    ACCELEROMETER_PERIOD_WRITE,
    ACCELEROMETER_PERIOD_UPDATE,
    UARTBLEMESSAGEID_MAX
};

namespace codal
{
class UartBle
{
public:
    NRF52Serial &serial;
    MicroBitAccelerometer &accelerometer;

    uint16_t accelerometerData[3];
    uint16_t accelerometerPeriodMs;

    UartBle(NRF52Serial &serial, MicroBitAccelerometer &accelerometer);

    void runRx();

    void sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength);

    void accelerometerDataHandler(MicroBitEvent const);

private:
    Fiber *rx_fiber;
};
} // namespace codal
