#include "NRF52Serial.h"

#define UARTBLE_ID 0xbeef

enum UartBleMessageId
{
    ID_RESERVED,
    BLE_CONNECTED,
    BLE_DISCONNECTED,
    ACCELEROMETER_DATA_UPDATE,
    ACCELEROMETER_PERIOD_WRITE,
    ACCELEROMETER_PERIOD_UPDATE,
    MAGNETOMETER_DATA_UPDATE,
    MAGNETOMETER_BEARING_UPDATE,
    MAGNETOMETER_PERIOD_WRITE,
    MAGNETOMETER_PERIOD_UPDATE,
    MAGNETOMETER_CALIBRATION_UPDATE,
    MAGNETOMETER_CALIBRATION_REQUESTED,
    PIN_DATA_WRITE,
    PIN_DATA_UPDATE,
    PIN_DATA_REQUEST,
    PIN_AD_CONFIGURATION_WRITE,
    PIN_IO_CONFIGURATION_WRITE,
    PIN_PWM_WRITE,
    UARTBLEMESSAGEID_MAX
};

struct PinDataWritePayload
{
    uint8_t length;
    uint8_t data[];
};

struct PinPwmWritePayload
{
    uint8_t length;
    uint8_t data[];
};

namespace codal
{
class UartBle
{
public:
    NRF52Serial &serial;

    uint16_t accelerometerData[3];
    uint16_t accelerometerPeriodMs;

    int16_t magnetometerData[3];
    uint16_t magnetometerBearing;
    uint16_t magnetometerPeriodMs;
    uint8_t magnetometerCalibration;

    PinDataWritePayload *pinData = NULL;
    uint32_t pinADConfiguration;
    uint32_t pinIOConfiguration;
    PinPwmWritePayload *pinPwmControl = NULL;

    UartBle(NRF52Serial &serial);

    void runRx();

    void sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength);

private:
    Fiber *rx_fiber;
};
} // namespace codal
