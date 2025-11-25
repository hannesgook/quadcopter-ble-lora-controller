# SPDX-FileCopyrightText: 2018 Brent Rubell for Adafruit Industries
# SPDX-License-Identifier: MIT

# Modifications and new code © 2025 Hannes Göök

# When text is received over BLE, it is transmitted over LoRa.

import time
import busio
from digitalio import DigitalInOut
import board
import adafruit_rfm9x
from bluezero import adapter, peripheral, device
import threading
import queue


# --- LoRa Setup ---
CS0 = DigitalInOut(board.CE1)
RESET0 = DigitalInOut(board.D22)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

rfm9x = None
try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS0, RESET0, 433.0)
    rfm9x.tx_power = 20  # dBm
    print("RFM9x: Detected and initialized")
except RuntimeError as error:
    print("RFM9x Error:", error)

# --- BLE UART Setup ---
UART_SERVICE = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
RX_CHARACTERISTIC = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
TX_CHARACTERISTIC = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'

class UARTDevice:
    tx_obj = None
    connected = False

    lora_queue = queue.Queue(maxsize=1)
    lora_thread = None
    running = True

    @classmethod
    def start_lora_thread(cls):
        if cls.lora_thread is None:
            t = threading.Thread(target=cls._lora_loop, daemon=True)
            cls.lora_thread = t
            t.start()

    @classmethod
    def _enqueue_latest(cls, text: str):
        while True:
            try:
                cls.lora_queue.put_nowait(text)
                break
            except queue.Full:
                try:
                    cls.lora_queue.get_nowait()
                except queue.Empty:
                    break

    @classmethod
    def _lora_loop(cls):
        last_send = 0.0
        while cls.running:
            text = cls.lora_queue.get()
            if text is None:
                break

            if not cls.connected:
                print("Dropped LoRa packet, BLE disconnected")
                continue

            now = time.time()
            dt = now - last_send
            target_period = 0.1  # 10 Hz
            if dt < target_period:
                time.sleep(target_period - dt)

            if not cls.connected:
                print("Skipped LoRa send, BLE disconnected after wait")
                continue

            if not rfm9x:
                print("RFM9x not initialized")
                continue

            try:
                rfm9x.send(text.encode("utf-8"))
                last_send = time.time()
                print("Sent over LoRa:", text)
            except Exception as e:
                print("LoRa send failed:", e)

    @classmethod
    def flush_queue(cls):
        try:
            with cls.lora_queue.mutex:
                cls.lora_queue.queue.clear()
        except Exception:
            pass

    @classmethod
    def on_connect(cls, ble_device: device.Device):
        cls.connected = True
        print("Connected to", ble_device.address)

    @classmethod
    def on_disconnect(cls, adapter_address, device_address):
        cls.connected = False
        cls.tx_obj = None
        cls.flush_queue()
        print("Disconnected from", device_address)

    @classmethod
    def uart_notify(cls, notifying, characteristic):
        if notifying:
            cls.tx_obj = characteristic
        else:
            cls.tx_obj = None

    @classmethod
    def update_tx(cls, value):
        if cls.tx_obj and cls.connected:
            cls.tx_obj.set_value(value)

    @classmethod
    def uart_write(cls, value, options):
        if not cls.connected:
            print("Ignoring RX, no BLE device connected")
            return

        raw = bytes(value)
        text = raw.decode('utf-8', errors='replace').strip()
        print("BLE RX:", text)

        cls._enqueue_latest(text)
        cls.update_tx(value)

def main(adapter_address):
    UARTDevice.start_lora_thread()

    ble_uart = peripheral.Peripheral(adapter_address, local_name='BLE-LoRa-Bridge')

    ble_uart.add_service(srv_id=1, uuid=UART_SERVICE, primary=True)

    # RX characteristic (BLE to Pi) NOT USED
    ble_uart.add_characteristic(
        srv_id=1, chr_id=1, uuid=RX_CHARACTERISTIC,
        value=[], notifying=False,
        flags=['write', 'write-without-response'],
        write_callback=UARTDevice.uart_write,
        read_callback=None,
        notify_callback=None
    )

    # TX characteristic (Pi to BLE)
    ble_uart.add_characteristic(
        srv_id=1, chr_id=2, uuid=TX_CHARACTERISTIC,
        value=[], notifying=False,
        flags=['notify'],
        notify_callback=UARTDevice.uart_notify,
        read_callback=None,
        write_callback=None
    )

    ble_uart.on_connect = UARTDevice.on_connect
    ble_uart.on_disconnect = UARTDevice.on_disconnect

    print("Starting BLE UART peripheral...")
    ble_uart.publish()


if __name__ == '__main__':
    try:
        main(list(adapter.Adapter.available())[0].address)
    finally:
        UARTDevice.running = False
        UARTDevice.lora_queue.put(None)
