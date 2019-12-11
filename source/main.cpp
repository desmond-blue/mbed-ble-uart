/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/gap/Gap.h"
#include "pretty_printer.h"

const static char DEVICE_NAME[] = "BLE Uart";

/* from Nordic Uart Service */
const uint8_t NUS_BASE_UUID[UUID::LENGTH_OF_LONG_UUID] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E};
const uint8_t BLE_UUID_NUS_SERVICE[UUID::LENGTH_OF_LONG_UUID] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
const uint8_t BLE_UUID_NUS_TX_CHARACTERISTIC[UUID::LENGTH_OF_LONG_UUID] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};
const uint8_t BLE_UUID_NUS_RX_CHARACTERISTIC[UUID::LENGTH_OF_LONG_UUID] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};

static events::EventQueue event_queue(/* event count */ 16 * EVENTS_EVENT_SIZE);

class BleUartDemo : ble::Gap::EventHandler {

public:
    /** Maximum length of data (in bytes) that the UART service module can transmit to the peer. */
    static const unsigned BLE_UART_SERVICE_MAX_DATA_LEN = (BLE_GATT_MTU_SIZE_DEFAULT - 3);

public:
    BleUartDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _led1(LED1, 1),
        _connected(false),
        _uart_uuid(UUID(NUS_BASE_UUID, UUID::LSB)),
        _adv_data_builder(_adv_buffer),
        receiveBuffer(),
        sendBuffer(),
        sendBufferIndex(0),
        numBytesReceived(0),
        receiveBufferIndex(0),
        txCharacteristic(/* UUID */ UUID(BLE_UUID_NUS_TX_CHARACTERISTIC, UUID::LSB),
                         /* Initial value */ receiveBuffer,
                         /* Value size */ 1,
                         /* Value capacity */ BLE_UART_SERVICE_MAX_DATA_LEN,
                         /* Properties */ GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
                         /* Descriptors */ NULL,
                         /* Num descriptors */ 0,
                         /* variable len */ true),
        rxCharacteristic(/* UUID */ UUID(BLE_UUID_NUS_RX_CHARACTERISTIC, UUID::LSB),
                         /* Initial value */ sendBuffer,
                         /* Value size */ 1,
                         /* Value capacity */ BLE_UART_SERVICE_MAX_DATA_LEN,
                         /* Properties */ GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE,
                         /* Descriptors */ NULL,
                         /* Num descriptors */ 0,
                         /* variable len */ true) {
            GattCharacteristic *charTable[] = {&txCharacteristic, &rxCharacteristic};
            GattService         uartService(UUID(BLE_UUID_NUS_SERVICE, UUID::LSB), charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
            _ble.gattServer().addService(uartService);
        }

    void start() {
        _ble.gap().setEventHandler(this);
        
        _ble.init(this, &BleUartDemo::on_init_complete);
        
        _event_queue.call_every(500, this, &BleUartDemo::blink);

        _event_queue.call_every(1000, this, &BleUartDemo::SendMessage);

        _event_queue.dispatch_forever();
    }

    uint16_t getTXCharacteristicHandle() {
        return txCharacteristic.getValueAttribute().getHandle();
    }

    void SendMessage() {
        uint8_t data[] = "hello";
        _uart->write(getTXCharacteristicHandle(), data, sizeof(data), false);
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        print_mac_address();

        _uart = &_ble.gattServer();
        _uart->onDataSent(this, &BleUartDemo::when_data_sent);
        _uart->onDataWritten(this, &BleUartDemo::when_data_written);

        start_advertising();
    }

    void start_advertising() {
        /* Create advertising parameters and payload */

        ble::AdvertisingParameters adv_parameters(
            ble::advertising_type_t::CONNECTABLE_UNDIRECTED,
            ble::adv_interval_t(ble::millisecond_t(1000))
        );

        _adv_data_builder.setFlags();
        _adv_data_builder.setLocalServiceList(mbed::make_Span(&_uart_uuid, 1));
        _adv_data_builder.setName(DEVICE_NAME);

        /* Setup advertising */

        ble_error_t error = _ble.gap().setAdvertisingParameters(
            ble::LEGACY_ADVERTISING_HANDLE,
            adv_parameters
        );

        if (error) {
            printf("_ble.gap().setAdvertisingParameters() failed\r\n");
            return;
        }

        error = _ble.gap().setAdvertisingPayload(
            ble::LEGACY_ADVERTISING_HANDLE,
            _adv_data_builder.getAdvertisingData()
        );

        if (error) {
            printf("_ble.gap().setAdvertisingPayload() failed\r\n");
            return;
        }

        /* Start advertising */

        error = _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

        if (error) {
            printf("_ble.gap().startAdvertising() failed\r\n");
            return;
        }

        printf("start advertising\r\n");
    }

    void blink(void) {
        _led1 = !_led1;
    }

private:
    /* Event handler */

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        _ble.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        _connected = false;
    }

    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            _connected = true;
        }
    }

private:
    /**
     * Handler called when a notification or an indication has been sent.
     */
    void when_data_sent(unsigned count)
    {
        printf("sent %u updates\r\n", count);
        sendBufferIndex = 0;
    }

    /**
     * Handler called after an attribute has been written.
     */
    void when_data_written(const GattWriteCallbackParams *e)
    {
        for (size_t i = 0; i < e->len; ++i) {
            printf("%c", e->data[i]);
        }

        printf("\r\n");
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    DigitalOut _led1;

    bool _connected;

    UUID _uart_uuid;

    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;

    /* UART service */
    uint8_t             receiveBuffer[BLE_UART_SERVICE_MAX_DATA_LEN]; /**< The local buffer into which we receive
                                                                       *   inbound data before forwarding it to the
                                                                       *   application. */

    uint8_t             sendBuffer[BLE_UART_SERVICE_MAX_DATA_LEN];    /**< The local buffer into which outbound data is
                                                                       *   accumulated before being pushed to the
                                                                       *   rxCharacteristic. */
    uint8_t             sendBufferIndex;
    uint8_t             numBytesReceived;
    uint8_t             receiveBufferIndex;

    GattServer* _uart;

    GattCharacteristic  txCharacteristic; /**< From the point of view of the external client, this is the characteristic
                                           *   they'd write into in order to communicate with this application. */
    GattCharacteristic  rxCharacteristic; /**< From the point of view of the external client, this is the characteristic
                                           *   they'd read from in order to receive the bytes transmitted by this
                                           *   application. */
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{    
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    BleUartDemo demo(ble, event_queue);       

    demo.start();

    return 0;
}

