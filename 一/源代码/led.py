import RPi.GPIO as GPIO
import time

# 使用物理引脚编号模式
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO_LED = 29  # 连接 蜂鸣器 的引脚

# 设置 GPIO_LED 为输出并设置初始状态为低电平
GPIO.setup(GPIO_LED, GPIO.OUT)
GPIO.output(GPIO_LED, GPIO.LOW)

try:
    for i in range(5):
        print(f"LED 闪烁第 {i+1} 次")
        GPIO.output(GPIO_LED, GPIO.HIGH)
        print("LED 引脚 29 输出高电平")
        time.sleep(0.5)
        GPIO.output(GPIO_LED, GPIO.LOW)
        print("LED 引脚 29 输出低电平")
        time.sleep(0.5)
except Exception as e:
    print(f"发生错误: {e}")
finally:
    print("清理 GPIO 设置")
    GPIO.cleanup()
