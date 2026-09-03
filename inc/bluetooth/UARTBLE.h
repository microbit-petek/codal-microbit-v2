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
    MAGNETOMETER_CALIBRATION_REQUEST,
    PIN_DATA_WRITE,
    PIN_DATA_UPDATE,
    PIN_DATA_REQUEST,
    PIN_AD_CONFIGURATION_WRITE,
    PIN_IO_CONFIGURATION_WRITE,
    PIN_PWM_WRITE,
    LED_DATA_WRITE,
    LED_DATA_UPDATE,
    LED_DATA_REQUEST,
    LED_TEXT_WRITE,
    LED_SCROLLING_DELAY_WRITE,
    PARTIAL_FLASHING_COMMAND,
    PARTIAL_FLASHING_REGION_INFO,
    PARTIAL_FLASHING_STATUS,
    TEMPERATURE_PERIOD_WRITE,
    TEMPERATURE_PERIOD_UPDATE,
    TEMPERATURE_DATA_UPDATE,
    UARTBLEMESSAGEID_MAX,

    INTERFACE_IDLE_REPORT = 0xaa,
    TARGET_IDLE_REPORT,
    INTERFACE_IDLE_REPORT_TO_BOUNCE,
};

#define UARTBLE_REPORT_IDLE UARTBLEMESSAGEID_MAX + 1

struct VariableLengthPayload
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

    VariableLengthPayload *pinData = NULL;
    uint32_t pinADConfiguration;
    uint32_t pinIOConfiguration;
    VariableLengthPayload *pinPwmControl = NULL;

    uint8_t ledData[5];
    uint16_t ledScrollDelay;
    VariableLengthPayload *ledRawText = NULL;

    VariableLengthPayload *partialFlashingMessage = NULL;

    int8_t temperatureData;
    uint16_t temperaturePeriodMs;

    UartBle(NRF52Serial &serial);

    void runRx();

    void sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength);

    void connected(bool isTarget);

private:
    Fiber *rx_fiber;

    bool isTarget = false;

    void updateVLP(VariableLengthPayload ** const vlp);

    void reportIdle(Event);

};
} // namespace codal
