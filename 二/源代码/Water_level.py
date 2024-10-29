import spidev
import time


class WaterLevel:
    def __init__(self, channel, refVolt, resolution, range):
        self.channel = channel
        self.refVolt = refVolt
        self.range = range
        self.resolution = resolution
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000

    def __getMcp3008(self):
        '''
        读取ADC转换得到的数字量
        :return: data, ADC转换结果，10位
        '''
        adc = self.spi.xfer2([1, (8 + self.channel) << 4, 0])  # SPI传输
        data = ((adc[1] & 3) << 8) + adc[2]  # 根据ADC分辨率调整
        return data

    def __toVolt(self, data):
        # 将data转换为电压
        return (data * self.refVolt) / (2 ** self.resolution - 1)

    def __toLevel(self, volts):
        # 将volts转换为液位
        return (volts / self.refVolt) * self.range  # 根据传感器的范围进行线性转换

    def read(self):
        '''
        根据传感器资料进行电压到液位的转换
        :return: 液面值level, 电压值volt
        '''
        data = self.__getMcp3008()
        volt = self.__toVolt(data)
        level = self.__toLevel(volt)
        return level, volt


if __name__ == '__main__':
    waterLevel = WaterLevel(1, 3.3, 10, 48)
    while True:
        level, volt = waterLevel.read()
        print('电压：{:.1f} V, 液面：{:.1f} cm'.format(volt, level))
        time.sleep(3)
