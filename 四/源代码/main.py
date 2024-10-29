# main.py

import threading
import time
import queue
import RPi.GPIO as GPIO
from config import (
    I2C_ADDR,
    GPIO_LED,
    GPIO_PHASE_A,
    GPIO_PHASE_B,
    GPIO_PHASE_C,
    GPIO_PHASE_D,
    GPIO_HEATER,
    LIGHT_UPDATE_INTERVAL,
    LIGHT_MAX_LUX,
    LIGHT_MIN_LUX,
    WATER_LEVEL_UPPER,
    WATER_LEVEL_LOWER,
    ADC_CHANNEL,
    ADC_REF_VOLT,
    ADC_RESOLUTION,
    WATER_LEVEL_RANGE,
    MOTOR_ROTATION_DELAY,
    MOTOR_DEGREES,
    TEMP_DEVICE_FILE,  # 将被自动检测
    TEMP_UPDATE_INTERVAL,
    TEMP_LOWER_LIMIT,
    TEMP_UPPER_LIMIT,
    ALINK_PRODUCT_KEY,
    ALINK_DEVICE_NAME,
    ALINK_DEVICE_SECRET,
    ALINK_SERVER,
    ALINK_POST_TOPIC
)
import smbus2 as smbus
import spidev
import math
from aliLink import linkiot, Alink
from mqttd import MQTT
import os

# 初始化GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# 设置加热器引脚
GPIO.setup(GPIO_HEATER, GPIO.OUT)
GPIO.output(GPIO_HEATER, GPIO.LOW)  # 初始关闭

# 全局停止事件，用于停止所有线程
stop_event = threading.Event()

# 命令队列，用于步进电机控制
motor_command_queue = queue.Queue()

# 锁用于保护状态变量
state_lock = threading.Lock()

# 当前水龙头状态，初始为关闭
faucet_open = False

# 当前光照强度
current_light = 0.0

# 当前水位
current_water_level = 0.0

# 当前LED亮度（0-100%）
current_led_brightness = 0.0

# 当前温度
current_temperature = 0.0

# 加热器状态
heater_state = "off"

# MQTT 发布状态
tap_state = "closed"  # "closed" 或 "open"
led_state = "off"  # "off" 或 "xx.xx%"


# hot_state 不需要独立变量，因为它与 heater_state 同步

# BH1750 光照传感器类
class Bh1750:
    def __init__(self, addr):
        self.ADDR = addr
        self.PWR_OFF = 0x00
        self.PWR_ON = 0x01
        self.RESET = 0x07
        self.THRES = 0x20
        self.bus = smbus.SMBus(1)

    def power_on(self):
        self.bus.write_byte(self.ADDR, self.PWR_ON)

    def power_off(self):
        self.bus.write_byte(self.ADDR, self.PWR_OFF)

    def reset(self):
        self.bus.write_byte(self.ADDR, self.RESET)

    def measure(self):
        try:
            self.power_on()
            self.bus.write_byte(self.ADDR, self.THRES)
            time.sleep(0.2)
            res = self.bus.read_word_data(self.ADDR, 0)
            res = ((res >> 8) & 0xff) | ((res << 8) & 0xff00)
            res = round(res / 1.2, 1)
            self.power_off()
            return res
        except Exception as e:
            print(f"[Bh1750] 测量光照强度失败: {e}")
            self.power_off()
            return None


# 水位传感器类
class WaterLevel:
    def __init__(self, channel, refVolt, resolution, range_cm):
        self.channel = channel
        self.refVolt = refVolt
        self.range = range_cm
        self.resolution = resolution
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1350000

    def __getMcp3008(self):
        adc = self.spi.xfer2([1, (8 + self.channel) << 4, 0])
        data = ((adc[1] & 3) << 8) + adc[2]
        return data

    def __toVolt(self, data):
        return (data * self.refVolt) / (2 ** self.resolution - 1)

    def __toLevel(self, volts):
        return (volts / self.refVolt) * self.range

    def read(self):
        try:
            data = self.__getMcp3008()
            volt = self.__toVolt(data)
            level = self.__toLevel(volt)
            return level, volt
        except Exception as e:
            print(f"[WaterLevel] 读取水位失败: {e}")
            return None, None


# 步进电机类
class SteppingMotor(threading.Thread):
    def __init__(self, pins, rotation_delay=MOTOR_ROTATION_DELAY):
        super().__init__()
        self.pins = pins  # (GPIO_PHASE_A, GPIO_PHASE_B, GPIO_PHASE_C, GPIO_PHASE_D)
        self.rotation_delay = rotation_delay
        self.pulse_sequence = [
            (1, 0, 0, 0),
            (1, 1, 0, 0),
            (0, 1, 0, 0),
            (0, 1, 1, 0),
            (0, 0, 1, 0),
            (0, 0, 1, 1),
            (0, 0, 0, 1),
            (1, 0, 0, 1)
        ]
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def degrees_to_steps(self, degrees):
        pulses_per_revolution = 4096
        steps = int(pulses_per_revolution * degrees / 360)
        return steps

    def set_step(self, pulse):
        p1, p2, p3, p4 = pulse
        GPIO.output(self.pins[0], p1)
        GPIO.output(self.pins[1], p2)
        GPIO.output(self.pins[2], p3)
        GPIO.output(self.pins[3], p4)
        time.sleep(self.rotation_delay)

    def rotate_motor(self, direction=0, degrees=MOTOR_DEGREES):
        steps = self.degrees_to_steps(degrees)
        for step in range(steps):
            if stop_event.is_set():
                break
            if direction == 0:
                pulse = self.pulse_sequence[step % len(self.pulse_sequence)]
            else:
                pulse = self.pulse_sequence[::-1][step % len(self.pulse_sequence)]
            self.set_step(pulse)
        # 关闭所有引脚
        for pin in self.pins:
            GPIO.output(pin, GPIO.LOW)

    def run(self):
        try:
            while not stop_event.is_set():
                try:
                    command = motor_command_queue.get(timeout=0.5)
                    if command['action'] == 'open':
                        print("[Motor] 执行打开水龙头的命令")
                        self.rotate_motor(direction=1)
                        # 更新水龙头状态
                        with state_lock:
                            global faucet_open, tap_state
                            faucet_open = True
                            tap_state = "open"
                    elif command['action'] == 'close':
                        print("[Motor] 执行关闭水龙头的命令")
                        self.rotate_motor(direction=0)
                        # 更新水龙头状态
                        with state_lock:
                            faucet_open = False
                            tap_state = "closed"
                except queue.Empty:
                    continue
        except Exception as e:
            print(f"[Motor] 发生错误: {e}")
        finally:
            for pin in self.pins:
                GPIO.output(pin, GPIO.LOW)


# LED 控制线程
class LightController(threading.Thread):
    def __init__(self, led_pin, light_sensor, max_lux, min_lux, update_interval=LIGHT_UPDATE_INTERVAL):
        super().__init__()
        self.led_pin = led_pin
        self.light_sensor = light_sensor
        self.max_lux = max_lux
        self.min_lux = min_lux
        self.update_interval = update_interval
        GPIO.setup(self.led_pin, GPIO.OUT)
        self.led = GPIO.PWM(self.led_pin, 1000)  # 1kHz
        self.led.start(0)  # 初始关闭

    def set_led_brightness(self, duty_cycle):
        self.led.ChangeDutyCycle(duty_cycle)
        print(f"[LightController] 设置LED亮度为: {duty_cycle:.2f}%")

    def map_lux_to_duty_cycle(self, lux):
        """
        使用对数映射将光照强度映射到 PWM 占空比。
        - 光照越强，LED 亮度越低。
        - 光照越弱，LED 亮度越高。
        """
        if lux is None:
            # 如果测量失败，保持当前亮度
            print("[LightController] 光照测量失败，保持当前亮度")
            return

        if lux >= self.max_lux:
            return 0.0  # 关闭LED
        elif lux <= self.min_lux:
            return 100.0  # 全亮
        else:
            # 对数映射
            normalized_lux = (lux - self.min_lux) / (self.max_lux - self.min_lux)
            # 防止对数函数输入为0
            normalized_lux = max(normalized_lux, 0.0001)
            # 使用对数函数进行映射
            duty_cycle = (math.log(normalized_lux * 9 + 1) / math.log(10)) * 100
            # 确保占空比在0-100之间
            duty_cycle = min(max(duty_cycle, 0.0), 100.0)
            return duty_cycle

    def run(self):
        global current_light, current_led_brightness, led_state
        try:
            while not stop_event.is_set():
                light_level = self.light_sensor.measure()
                if light_level is not None:
                    print(f"[LightController] 当前光照强度: {light_level} lx")
                    duty_cycle = self.map_lux_to_duty_cycle(light_level)
                    if duty_cycle is not None:
                        self.set_led_brightness(duty_cycle)
                        with state_lock:
                            current_light = light_level
                            current_led_brightness = duty_cycle
                            led_state = f"{duty_cycle:.2f}"
                time.sleep(self.update_interval)
        except Exception as e:
            print(f"[LightController] 发生错误: {e}")
        finally:
            self.led.stop()
            GPIO.output(self.led_pin, GPIO.LOW)
            print("[LightController] 停止LED控制线程")


# 水位控制线程
class WaterLevelController(threading.Thread):
    def __init__(self, water_sensor, upper_threshold, lower_threshold):
        super().__init__()
        self.water_sensor = water_sensor
        self.upper_threshold = upper_threshold
        self.lower_threshold = lower_threshold

    def run(self):
        global current_water_level, tap_state
        try:
            while not stop_event.is_set():
                level, volt = self.water_sensor.read()
                if level is not None and volt is not None:
                    print(f"[WaterLevelController] 当前水位: {level:.1f} cm, 电压: {volt:.2f} V")
                    with state_lock:
                        current_water_level = level

                    if level < self.lower_threshold:
                        with state_lock:
                            current_state = faucet_open
                        if not current_state:
                            print("[WaterLevelController] 水位低于下限，打开水龙头")
                            motor_command_queue.put({'action': 'open'})
                        else:
                            print("[WaterLevelController] 水龙头已打开，无需操作")
                    elif self.lower_threshold <= level <= self.upper_threshold:
                        with state_lock:
                            current_state = faucet_open
                        if current_state:
                            print("[WaterLevelController] 水位在正常范围，关闭水龙头")
                            motor_command_queue.put({'action': 'close'})
                        else:
                            print("[WaterLevelController] 水龙头已关闭，无需操作")
                    elif level > self.upper_threshold:
                        with state_lock:
                            current_state = faucet_open
                        if current_state:
                            print("[WaterLevelController] 水位高于上限，关闭水龙头")
                            motor_command_queue.put({'action': 'close'})
                        else:
                            print("[WaterLevelController] 水龙头已关闭，无需操作")
                time.sleep(5)  # 每5秒检测一次
        except Exception as e:
            print(f"[WaterLevelController] 发生错误: {e}")
        finally:
            print("[WaterLevelController] 停止水位控制线程")


# 温度传感器类
class TemperatureSensor:
    def __init__(self, device_file):
        self.device_file = device_file

    def read_temp_raw(self):
        try:
            with open(self.device_file, 'r') as f:
                lines = f.readlines()
            return lines
        except Exception as e:
            print(f"[TemperatureSensor] 读取温度原始数据失败: {e}")
            return None

    def read_temp(self):
        lines = self.read_temp_raw()
        if lines is None:
            return None
        while lines[0].strip()[-3:] != 'YES':
            print("[TemperatureSensor] CRC校验失败，重试...")
            time.sleep(0.2)
            lines = self.read_temp_raw()
            if lines is None:
                return None
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            try:
                temp_c = float(temp_string) / 1000.0
                return temp_c
            except ValueError:
                print("[TemperatureSensor] 温度转换失败")
                return None
        else:
            print("[TemperatureSensor] 未找到温度数据")
            return None


# 温度控制线程
class TemperatureController(threading.Thread):
    def __init__(self, temp_sensor, lower_limit, upper_limit, update_interval=TEMP_UPDATE_INTERVAL):
        super().__init__()
        self.temp_sensor = temp_sensor
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.update_interval = update_interval

    def run(self):
        global current_temperature, heater_state
        try:
            while not stop_event.is_set():
                temp = self.temp_sensor.read_temp()
                if temp is not None:
                    print(f"[TemperatureController] 当前温度: {temp:.2f} °C")
                    with state_lock:
                        current_temperature = temp

                    if temp < self.lower_limit:
                        if heater_state != "on":
                            print("[TemperatureController] 温度低于下限，打开加热器")
                            GPIO.output(GPIO_HEATER, GPIO.HIGH)
                            heater_state = "on"
                    else:
                        if heater_state != "off":
                            print("[TemperatureController] 温度达到或高于下限，关闭加热器")
                            GPIO.output(GPIO_HEATER, GPIO.LOW)
                            heater_state = "off"
                else:
                    print("[TemperatureController] 温度读取失败")
                time.sleep(self.update_interval)
        except Exception as e:
            print(f"[TemperatureController] 发生错误: {e}")
        finally:
            GPIO.output(GPIO_HEATER, GPIO.LOW)
            print("[TemperatureController] 停止温度控制线程")


# PubSub 类
class PubSub(threading.Thread):
    '''
    树莓派端，与阿里云通信
    '''

    def __init__(self, product_key, device_name, device_secret, topic):
        super().__init__()
        self.product_key = product_key
        self.device_name = device_name
        self.device_secret = device_secret
        self.topic = topic

        # 获取连接信息
        Server, ClientId, userName, Password = linkiot(
            DeviceName=self.device_name,
            ProductKey=self.product_key,
            DeviceSecret=self.device_secret
        )
        self.mqtt = MQTT(Server, ClientId, userName, Password)
        self.mqtt.begin(self.__on_message, self.__on_connect)

    # 消息回调（云端下发消息的回调函数）
    def __on_message(self, client, userdata, msg):
        # 这里可以处理云端下发的消息
        print(f"[PubSub] 收到消息: {msg.topic} {msg.payload.decode()}")

    # 连接回调
    def __on_connect(self, client, userdata, flags, rc):
        '''
        :param client: the client instance for this callback
        :param userdata: the private user data as set in Client()
        :param flags: response flags sent by the broker
        :param rc: the connection result
            0: Connection successful
            1: Connection refused - incorrect protocol version
            2: Connection refused - invalid client identifier
            3: Connection refused - server unavailable
            4: Connection refused - bad username or password
            5: Connection refused - not authorised
        '''
        if rc == 0:
            print("Connected to Aliyun IoT successfully!")
        else:
            print(f"Connection to Aliyun IoT refused with result code: {rc}")

    def pub(self, light, waterLevel, ledState, tapState, temperature, hotState):
        # 构建与云端模型一致的消息结构
        msg = {
            'light': light,
            'waterLevel': waterLevel,
            'LED': ledState,
            'tap': tapState,
            'hot': hotState,
            'temperature': temperature
        }
        JsonMsg = Alink(msg)
        self.mqtt.push(self.topic, JsonMsg)  # 向阿里云IOT推送数据
        print(f"[PubSub] msg to Aliyun Cloud: {msg}")

    def run(self):
        # 保持线程运行
        while not stop_event.is_set():
            time.sleep(1)
        print("[PubSub] 停止PubSub线程")


# 发布数据线程
class Publisher(threading.Thread):
    def __init__(self, pubsub_instance, interval=5):
        super().__init__()
        self.pubsub = pubsub_instance
        self.interval = interval

    def run(self):
        try:
            while not stop_event.is_set():
                with state_lock:
                    light = current_light
                    waterLevel = current_water_level
                    ledState = float(current_led_brightness) if current_led_brightness else 0.0
                    tapState = 1 if tap_state == "open" else 0
                    hotState = 1 if heater_state == "on" else 0
                    temperature = current_temperature
                self.pubsub.pub(light, waterLevel, ledState, tapState, temperature, hotState)
                time.sleep(self.interval)
        except Exception as e:
            print(f"[Publisher] 发生错误: {e}")
        finally:
            print("[Publisher] 停止发布线程")


# 主程序
def main():
    # 自动检测DS18B20设备文件
    devices = [d for d in os.listdir('/sys/bus/w1/devices/') if d.startswith('28-')]
    if not devices:
        print("[Main] 未找到DS18B20温度传感器，请检查连接和1-Wire接口配置。")
        return
    device_file = f"/sys/bus/w1/devices/{devices[0]}/w1_slave"
    print(f"[Main] 使用温度传感器设备文件: {device_file}")

    # 创建传感器实例
    light_sensor = Bh1750(I2C_ADDR)
    water_sensor = WaterLevel(ADC_CHANNEL, ADC_REF_VOLT, ADC_RESOLUTION, WATER_LEVEL_RANGE)
    temp_sensor = TemperatureSensor(device_file)

    # 创建设备实例
    led_controller = LightController(GPIO_LED, light_sensor, LIGHT_MAX_LUX, LIGHT_MIN_LUX)
    motor = SteppingMotor((GPIO_PHASE_A, GPIO_PHASE_B, GPIO_PHASE_C, GPIO_PHASE_D))
    water_controller = WaterLevelController(water_sensor, WATER_LEVEL_UPPER, WATER_LEVEL_LOWER)
    temp_controller = TemperatureController(temp_sensor, TEMP_LOWER_LIMIT, TEMP_UPPER_LIMIT)

    # 初始化PubSub
    pubsub = PubSub(
        product_key=ALINK_PRODUCT_KEY,
        device_name=ALINK_DEVICE_NAME,
        device_secret=ALINK_DEVICE_SECRET,
        topic=ALINK_POST_TOPIC
    )

    # 创建设备实例
    publisher = Publisher(pubsub, interval=2)

    # 启动线程
    led_controller.start()
    motor.start()
    water_controller.start()
    temp_controller.start()
    pubsub.start()
    publisher.start()

    try:
        while not stop_event.is_set():
            time.sleep(1)  # 主线程保持运行
    except KeyboardInterrupt:
        print("\n[Main] 程序中断，准备退出...")
        stop_event.set()
    finally:
        # 等待所有线程完成
        led_controller.join()
        water_controller.join()
        motor.join()
        temp_controller.join()
        pubsub.join()
        publisher.join()
        GPIO.cleanup()
        print("[Main] GPIO 设置已清理，程序退出。")


if __name__ == "__main__":
    main()
