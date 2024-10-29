from config import I2C_ADDR
import smbus2 as smbus
import time


class Bh1750:

    def __init__(self, addr):
        # BH1750地址
        self.ADDR = addr
        # 控制字
        self.PWR_OFF = 0x00  # 关机
        self.PWR_ON = 0x01  # 开机
        self.RESET = 0x07  # 重置
        self.CHRES = 0x10  # 持续高分辨率检测
        self.CHRES2 = 0x11  # 持续高分辨率模式2检测
        self.CLHRES = 0x13  # 持续低分辨率检测
        self.THRES = 0x20  # 一次高分辨率
        self.THRES2 = 0x21  # 一次高分辨率模式2
        self.TLRES = 0x23  # 一次低分辨率
        # 初始化I2C
        self.bus = smbus.SMBus(1)

    def power_on(self):
        self.bus.write_byte(self.ADDR, self.PWR_ON)

    def power_off(self):
        self.bus.write_byte(self.ADDR, self.PWR_OFF)

    def reset(self):
        self.bus.write_byte(self.ADDR, self.RESET)

    def measure(self):
        # 一次测量
        self.power_on()
        self.bus.write_byte(self.ADDR, self.THRES)
        time.sleep(0.2)
        res = self.bus.read_word_data(self.ADDR, 0)
        res = ((res >> 8) & 0xff) | ((res << 8) & 0xff00)
        res = round(res / 1.2, 1)
        self.power_off()
        return res


if __name__ == '__main__':
    light = Bh1750(I2C_ADDR)
    while True:
        res = light.measure()
        print(f"光照强度: {res} lx")
        time.sleep(3)
