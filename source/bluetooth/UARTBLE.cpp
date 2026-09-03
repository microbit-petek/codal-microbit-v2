#include "UARTBLE.h"
#include "CodalFiber.h"
#include "EventModel.h"
#include "Timer.h"

using namespace codal;


UartBle::UartBle(NRF52Serial &serial) : serial(serial) {
    EventModel::defaultEventBus->listen(UARTBLE_ID, UARTBLE_REPORT_IDLE, this, &UartBle::reportIdle);
    system_timer_event_every(1000, UARTBLE_ID, UARTBLE_REPORT_IDLE);
}

void UartBle::sendMessage(uint8_t const id, void const *const payload, uint8_t const payloadLength)
{
    uint8_t header[] = {0xde, 0xad};
    serial.send(header, 2);

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
        if (0xde == serial.read() && 0xad == serial.read())
        {
            
            uint8_t rawId = serial.read();


            if (rawId >= UARTBLEMESSAGEID_MAX && rawId != INTERFACE_IDLE_REPORT_TO_BOUNCE)
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
            case PIN_DATA_WRITE:
            case PIN_DATA_UPDATE:
            {
                updateVLP(&pinData);
                break;
            }
            case PIN_AD_CONFIGURATION_WRITE:
            {
                serial.read((uint8_t *)&pinADConfiguration, sizeof(pinADConfiguration));
                break;
            }
            case PIN_IO_CONFIGURATION_WRITE:
            {
                serial.read((uint8_t *)&pinIOConfiguration, sizeof(pinIOConfiguration));
                break;
            }
            case PIN_PWM_WRITE:
            {
                updateVLP(&pinPwmControl);
                break;
            }
            case LED_DATA_WRITE:
            case LED_DATA_UPDATE:
            {
                serial.read(ledData, sizeof(ledData));
                break;
            }
            case LED_TEXT_WRITE:
            {
                updateVLP(&ledRawText);
                break;
            }
            case LED_SCROLLING_DELAY_WRITE:
            {
                serial.read((uint8_t *)&ledScrollDelay, sizeof(ledScrollDelay));
                break;
            }
            case PARTIAL_FLASHING_COMMAND:
            {
                updateVLP(&partialFlashingMessage);
                break;
            }
            case PARTIAL_FLASHING_REGION_INFO:
            {
                if (partialFlashingMessage != NULL)
                {
                    delete partialFlashingMessage;
                    partialFlashingMessage = NULL;
                }

                uint8_t const regionInfoSize = 18;
                partialFlashingMessage = (VariableLengthPayload *)new uint8_t[regionInfoSize + sizeof(VariableLengthPayload)];
                partialFlashingMessage->length = regionInfoSize;
                serial.read(partialFlashingMessage->data, regionInfoSize);
                break;
            }
            case PARTIAL_FLASHING_STATUS:
            {
                if (partialFlashingMessage != NULL)
                {
                    delete partialFlashingMessage;
                    partialFlashingMessage = NULL;
                }

                uint8_t const statusSize = 3;
                partialFlashingMessage = (VariableLengthPayload *)new uint8_t[statusSize + sizeof(VariableLengthPayload)];
                partialFlashingMessage->length = statusSize;
                serial.read(partialFlashingMessage->data, statusSize);
                break;
            }
            case TEMPERATURE_PERIOD_WRITE:
            case TEMPERATURE_PERIOD_UPDATE:
            {
                serial.read((uint8_t *)&temperaturePeriodMs, sizeof(temperaturePeriodMs));
                break;
            }
            case TEMPERATURE_DATA_UPDATE:
            {
                temperatureData = serial.read();
                break;
            }

            // Diagnostic messages
            case INTERFACE_IDLE_REPORT_TO_BOUNCE:
            {
                uint16_t data;
                serial.read((uint8_t *)&data, sizeof(data));
                sendMessage(INTERFACE_IDLE_REPORT, &data, sizeof(data));
                continue;
            }

            // Payload-less messages
            case MAGNETOMETER_CALIBRATION_REQUEST:
            case PIN_DATA_REQUEST:
            case LED_DATA_REQUEST:
            {
                break;
            }
            case BLE_CONNECTED:
            {
                isTarget = true;
                break;
            }
            case BLE_DISCONNECTED:
            {
                break;
            }

            // Reserved IDs
            case UARTBLEMESSAGEID_MAX:
            case ID_RESERVED:
            {
                continue;
            }
            }

            Event(UARTBLE_ID, rawId);
        }
    }
}

void UartBle::updateVLP(VariableLengthPayload ** const vlp)
{
    if (NULL != *vlp)
    {
        delete[] *vlp;
        *vlp = NULL;
    }

    uint8_t const length = serial.read();
    *vlp = (VariableLengthPayload *)new uint8_t[length + sizeof(length)];
    (*vlp)->length = length;
    serial.read((*vlp)->data, length);
}

void UartBle::reportIdle(Event)
{
    uint8_t const id = isTarget ? TARGET_IDLE_REPORT : INTERFACE_IDLE_REPORT_TO_BOUNCE;
    uint16_t const idleTime = scheduler_get_idle_time_percentage_bps();
    sendMessage(id, &idleTime, sizeof(idleTime));
}

void UartBle::connected(bool yes)
{
    isTarget = yes;
}
