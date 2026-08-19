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
            case MAGNETOMETER_CALIBRATION_REQUEST:
            {
                break;
            }
            case PIN_DATA_WRITE:
            case PIN_DATA_UPDATE:
            {
                if (NULL != pinData)
                {
                    delete[] pinData;
                    pinData = NULL;
                }

                uint8_t const length = serial.read();
                pinData = (PinDataWritePayload *)new uint8_t[length + sizeof(length)];
                pinData->length = length;
                serial.read(pinData->data, length);
                break;
            }
            case PIN_DATA_REQUEST:
            {
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
                if (NULL != pinPwmControl)
                {
                    delete[] pinPwmControl;
                    pinPwmControl = NULL;
                }

                uint8_t const length = serial.read();
                pinPwmControl = (PinPwmWritePayload *)new uint8_t[length + sizeof(length)];
                pinPwmControl->length = length;
                serial.read(pinPwmControl->data, length);
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

            Event(UARTBLE_ID, rawId);
        }
    }
}
