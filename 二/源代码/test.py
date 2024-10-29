import spidev
import time


class MCP3008:
    def __init__(self, channel):
        self.channel = channel  # ADC通道
        self.spi = spidev.SpiDev()  # 实例化SPI
        self.spi.open(0, 0)  # 连接SPI设备0.0
        self.spi.max_speed_hz = 1350000  # 设置SPI通信速率

    def read_adc(self):
        # 读取ADC数据
        adc = self.spi.xfer2([1, (8 + self.channel) << 4, 0])  # SPI传输
        data = ((adc[1] & 3) << 8) + adc[2]  # 组合得到10位数据
        return data

    def close(self):
        self.spi.close()  # 关闭SPI连接


if __name__ == '__main__':
    adc_channel = 0  # 要测试的通道
    mcp = MCP3008(adc_channel)

    try:
        while True:
            data = mcp.read_adc()  # 读取ADC值
            volt = (data * 3.3) / 1023
            print(f"通道 {adc_channel} 的ADC值: {data}, 电压: {volt:.2f} V")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        mcp.close()  # 确保在退出时关闭SPI连接
