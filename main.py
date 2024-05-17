#
# *@author s2210033 大庭　
#

from machine import Pin, I2C
import time
import struct

# bus = I2C(0, scl=Pin(33, pull=Pin.PULL_UP), sda=Pin(32, pull=Pin.PULL_UP), freq=400000, timeout=1000000)
bus = I2C(0, scl=Pin(33), sda=Pin(32), freq=400000)
bmp180_i2c_addr = 0x77
SCD41_i2c_addr = 0x62
RPR_ADDR = 0x38

# def read_chip_id(bus):
#     bmp180_reg_chip_id = b'\xd0'
#     bus.writeto(bmp180_i2c_addr, bmp180_reg_chip_id, False)
#     chip_id_bmp = bus.readfrom(bmp180_i2c_addr, 1)

#     SCD41_reg_chip_id = b'\xd0'
#     bus.writeto(SCD41_i2c_addr, SCD41_reg_chip_id, False)
#     chip_id_SCD = bus.readfrom(SCD41_i2c_addr, 1)

#     if chip_id_bmp[0] == 0x55:
#         print("chip id is 0x55, BMP180 detected")
#     elif chip_id_SCD[0] == 0x56:
#         print("chip id is 0x56, SCD41 detected")


def bmp180_read_coefficients(bus) -> Bytes:
    bmp180_coef_reg_base = b'\xaa'
    bmp180_coef_size = 22

    bus.writeto(bmp180_i2c_addr, bmp180_coef_reg_base, False)
    coefs = bus.readfrom(bmp180_i2c_addr, bmp180_coef_size)

    # print(f"bmp coefficients: {coefs=}")
    return coefs


def bmp180_perform_measurement(bus, command: Bytes, ms: int) -> Bytes:
    bmp180_reg_out_msb = b'\xf6'

    bus.writeto(bmp180_i2c_addr, command, True)
    time.sleep_ms(ms)

    bus.writeto(bmp180_i2c_addr, bmp180_reg_out_msb, False)
    out = bus.readfrom(bmp180_i2c_addr, 3)

    # print(f"raw output: {[hex(x) for x in out]}")
    return out


def bmp180_read_temperature(bus) -> int:
    bmp180_cmd_meas_temp = b'\xf4\x2e'

    return bmp180_perform_measurement(bus, bmp180_cmd_meas_temp, 5)


def bmp180_read_pressure(bus) -> int:
    bmp180_cmd_meas_temp = b'\xf4\xf4'

    return bmp180_perform_measurement(bus, bmp180_cmd_meas_temp, 26)


def compute(coef, raw_temp, raw_press):
    # print("data computation")
    UT = struct.unpack_from(">h", raw_temp)[0]
    oss = 3
    UP = raw_press[0] << 16 | raw_press[1] << 8 | raw_press[2]
    UP = UP >> (8 - oss)

    AC1 = struct.unpack_from(">h", coef)[0]
    AC2 = struct.unpack_from(">h", coef, 2)[0]
    AC3 = struct.unpack_from(">h", coef, 4)[0]
    AC4 = struct.unpack_from(">H", coef, 6)[0]
    AC5 = struct.unpack_from(">H", coef, 8)[0]
    AC6 = struct.unpack_from(">H", coef, 10)[0]
    B1 = struct.unpack_from(">h", coef, 12)[0]
    B2 = struct.unpack_from(">h", coef, 14)[0]
    MB = struct.unpack_from(">h", coef, 16)[0]
    MC = struct.unpack_from(">h", coef, 18)[0]
    MD = struct.unpack_from(">h", coef, 20)[0]

    # print(f"{UT=}, {UP=}")
    # print(f"{AC1=}, {AC2=}, {AC3=}, {AC4=}, {AC5=}, {AC6=}")
    # print(f"{B1=}, {B2=}, {MB=}, {MC=}, {MD=}")

    # compute temperature
    X1 = (UT - AC6) * AC5 // 0x8000
    X2 = MC * 0x0800 // (X1 + MD)
    B5 = X1 + X2
    T = (B5 + 8) // 0x0010
    # T is in 0.1C units

    # compute pressure
    B6 = B5 - 4000
    X1 = (B2 * (B6 * B6 // (1 << 12))) // (1 < 11)
    X2 = AC2 * B6 // (1 << 11)
    X3 = X1 + X2
    B3 = (((AC1 * 4 + X3) << oss) + 2) // 4
    X1 = AC3 * B6 // (1 << 13)
    X2 = (B1 * (B6 * B6 // (1 << 12))) // (1 << 16)
    X3 = ((X1 + X2) + 2) // 4

    # unsigned longs here, check later
    B4 = AC4 * (X3 + 32768) // (1 << 15)
    B7 = (UP - B3) * (50000 >> 3)
    if B7 < 0x80000000:
        p = (B7 * 2) // B4
    else:
        p = (B7 // B4) * 2
    X1 = (p // 256) * (p // 256)
    X1 = (X1 * 3038) // (1 << 16)
    X2 = (-7357 * p) // (1 << 16)
    p = p + (X1 + X2 + 3791) // 16
    # print(f"measured temperature: {T / 10} ")
    print(f"air pressure: {p/100} hPa , temp: {T / 10}")


def scd41_stop_periodic_measurements():
    write_buf = bytearray([0x3f, 0x86])
    bus.writeto(SCD41_i2c_addr, write_buf)
    time.sleep(1)


def scd41_start_periodic_measurements():
    write_buf = bytearray([0x21, 0xb1])
    bus.writeto(SCD41_i2c_addr, write_buf)
    time.sleep(1)


def scd41_get_data_ready_status():
    write_buf = bytearray([0xe4, 0xb8])
    bus.writeto(SCD41_i2c_addr, write_buf)
    read_buf = bus.readfrom(SCD41_i2c_addr, 3)

    answer = int.from_bytes(read_buf[:2], 'big')
    return (answer & 0x07ff) != 0


def scd41_read_measurement():
    cmd = bytearray([0xec, 0x05])
    bus.writeto(SCD41_i2c_addr, cmd)
    measurements = bus.readfrom(SCD41_i2c_addr, 9)
    return measurements


def sensirion_common_generate_crc(data):
    CRC8_POLYNOMIAL = 0x31
    crc = 0xff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ CRC8_POLYNOMIAL
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def scd41_is_data_crc_correct(raw):
    for i in range(3):
        if sensirion_common_generate_crc(raw[i * 3:(i + 1) * 3]) != 0:
            print("SCD41: CRC ERROR at word number", i)
            return False
    return True


def scd41_calculate_and_show_data(raw):
    co2 = int.from_bytes(raw[0:2], 'big')
    raw_temperature = int.from_bytes(raw[3:5], 'big')
    raw_humidity = int.from_bytes(raw[6:8], 'big')

    temperature = -45 + 175 * (raw_temperature / 65535.0)
    humidity = 100 * (raw_humidity / 65535.0)

    print(f"SCD41: CO2: {co2} (ppm), temperature: {temperature:.2f} C, humidity: {humidity:.2f} %")


def scd41_poll():
    if not scd41_get_data_ready_status():
        print("SCD41: no new data available")
        return

    raw_measurements = scd41_read_measurement()

    if not scd41_is_data_crc_correct(raw_measurements):
        print("SCD41: crc error!")
        return

    scd41_calculate_and_show_data(raw_measurements)


def scd41_init():
    scd41_stop_periodic_measurements()
    time.sleep(1)
    scd41_start_periodic_measurements()
    print("SCD41: initialization finished")


def rpr_SYSTEM_CONTROL():
    bus.writeto(RPR_ADDR, bytes([0x40, 0x80]))


def rpr_MODE_CONTROL():
    bus.writeto(RPR_ADDR, bytes([0x41, 0x8a]))


def rpr_ALS_CONTROL():  # Gainの調整　
    bus.writeto(RPR_ADDR, bytes([0x42, 0x02]))

# 光センサーからデータを読み取る


def READ_ALS_DATA():
    # レジスタからデータを読み取り
    bus.writeto(RPR_ADDR, bytes([0x46, 0x01]))
    data = bus.readfrom(RPR_ADDR, 2)
    als_value = (data[1] << 8) | data[0]
    return als_value


def rpr_main():
    # 光センサーからデータを読み取る
    als_value = READ_ALS_DATA()
    print("ALS Reading: {} lux".format(als_value))


rpr_SYSTEM_CONTROL()
rpr_MODE_CONTROL()
rpr_ALS_CONTROL()
scd41_init()
# read_chip_id()
coef = bmp180_read_coefficients(bus)

start_time = time.time()
while time.time() - start_time < 15:
    scd41_poll()
    time.sleep(1)

start_time = time.time()
while time.time() - start_time < 15:
    raw_temp = bmp180_read_temperature(bus)
    raw_press = bmp180_read_pressure(bus)
    compute(coef, raw_temp, raw_press)
    time.sleep(1)

start_time = time.time()
while time.time() - start_time < 15:
    rpr_main()
    time.sleep(1)
