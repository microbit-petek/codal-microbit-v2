#include "UARTBLE.h"

using namespace codal;

UartBle::UartBle(NRF52Serial &serial) : serial(serial) {}

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

            // Payload-less messages
            case MAGNETOMETER_CALIBRATION_REQUEST:
            case PIN_DATA_REQUEST:
            case LED_DATA_REQUEST:
            case BLE_CONNECTED:
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
