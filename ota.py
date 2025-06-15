# ota_ble_update_ubuntu.py
import asyncio
import logging
from bleak import BleakScanner, BleakClient

logging.basicConfig(level=logging.DEBUG)

DEVICE_ADDR    = "F0:24:F9:45:5E:22"
CMD_CHAR_UUID  = "00008022-0000-1000-8000-00805f9b34fb"
DATA_CHAR_UUID = "00008020-0000-1000-8000-00805f9b34fb"
CHUNK_SIZE     = 512

async def run():
    print(f"Procurando dispositivo {DEVICE_ADDR}...")
    device = await BleakScanner.find_device_by_address(DEVICE_ADDR, timeout=20.0)
    if not device:
        print("Dispositivo não encontrado. Verifique o MAC e se está desconectado no SO.")
        return

    async with BleakClient(device) as client:
        if not await client.is_connected():
            print("Falha na conexão GATT")
            return
        print("Conectado! Resolvendo serviços...")

        # chama get_services() e conta quantos serviços existem
        services = await client.get_services()
        count = len(services.services)               # opção 1
        # — ou —
        # count = len(list(services))                # opção 2
        print(f"{count} serviços resolvidos")

        # START
        await client.write_gatt_char(CMD_CHAR_UUID, b"START")
        print("START enviado — iniciando transferência")

        total = 0
        with open("firmware.bin", "rb") as f:
            while chunk := f.read(CHUNK_SIZE):
                await client.write_gatt_char(DATA_CHAR_UUID, chunk)
                total += len(chunk)
                if total % (CHUNK_SIZE * 10) == 0:
                    print(f"  {total} bytes enviados")
        print(f"Envio completo: {total} bytes")

        # END
        await client.write_gatt_char(CMD_CHAR_UUID, b"END")
        print("END enviado — aguarde o reboot do ESP32")

if __name__ == "__main__":
    asyncio.run(run())

