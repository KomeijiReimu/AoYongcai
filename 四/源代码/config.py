# config.py

# BH1750 I2C 地址
I2C_ADDR = 0x23

GPIO_LED = 11  # LED 引脚
GPIO_PHASE_A = 32  # 步进电机相位 A
GPIO_PHASE_B = 36  # 步进电机相位 B
GPIO_PHASE_C = 38  # 步进电机相位 C
GPIO_PHASE_D = 40  # 步进电机相位 D
GPIO_HEATER = 12  # 加热器引脚（模拟）

# 光照配置
LIGHT_UPDATE_INTERVAL = 0.5  # 光照检测间隔（秒）
LIGHT_MAX_LUX = 1500.0  # 最大光照强度（lx）
LIGHT_MIN_LUX = 0.0  # 最小光照强度（lx）

# 水位门限（单位：cm）
WATER_LEVEL_UPPER = 45.0  # 液位上限
WATER_LEVEL_LOWER = 5  # 液位下限

# 水位传感器配置
ADC_CHANNEL = 1  # MCP3008 的通道
ADC_REF_VOLT = 3.3  # 参考电压
ADC_RESOLUTION = 10  # ADC 分辨率（位）
WATER_LEVEL_RANGE = 48.0  # 水位传感器量程（单位：cm）

# 步进电机配置
MOTOR_ROTATION_DELAY = 0.001  # 步进电机旋转延迟（秒）
MOTOR_DEGREES = 90  # 每次旋转的角度（度）

# 温度传感器配置
TEMP_DEVICE_FILE = '/sys/bus/w1/devices/28-00000038ad25/w1_slave'
TEMP_UPDATE_INTERVAL = 0.5  # 温度检测间隔（秒）
TEMP_LOWER_LIMIT = 25  # 温度下限（度）
TEMP_UPPER_LIMIT = 30.0  # 温度上限（度）

# 阿里云 IoT 平台配置
ALINK_PRODUCT_KEY = 'k1v5jOtSrMb'
ALINK_DEVICE_NAME = 'IOT_319'
ALINK_DEVICE_SECRET = 'b2f48e2ed8538e37a880a4cac78cf424'
ALINK_SERVER = 'iot-as-mqtt.cn-shanghai.aliyuncs.com'

# MQTT 主题配置
ALINK_POST_TOPIC = f'/sys/{ALINK_PRODUCT_KEY}/{ALINK_DEVICE_NAME}/thing/event/property/post'
