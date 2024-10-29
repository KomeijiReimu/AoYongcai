import RPi.GPIO as GPIO
import time

GPIO_LED = 7
FREQUENCY = 1000  # PWM频率，单位为Hz

GPIO.setmode(GPIO.BOARD)
GPIO.setup(GPIO_LED, GPIO.OUT)


class Led:
    def __init__(self, pin, frequency=1000):
        """
        初始化LED对象，设置PWM
        :param pin: GPIO引脚号（BOARD编号）
        :param frequency: PWM频率
        """
        self.pin = pin
        self.pwm = GPIO.PWM(self.pin, frequency)
        self.pwm.start(0)  # 初始占空比为0，即LED关闭

    def set_brightness(self, level):
        """
        设置LED亮度等级
        :param level: 亮度等级（0到9）
        """
        if level < 0 or level > 9:
            raise ValueError("亮度等级必须在0到9之间")
        duty_cycle = (level / 9) * 100  # 将等级映射到0-100%的占空比
        self.pwm.ChangeDutyCycle(duty_cycle)
        print(f"设置亮度等级: {level}, 占空比: {duty_cycle:.2f}%")

    def cleanup(self):
        """
        清理PWM和GPIO设置
        """
        self.pwm.stop()
        GPIO.cleanup()


def trapezoidal_brightness_change(led):
    """
    实现亮度梯形变化，从0逐渐增加到9，然后逐渐减少到0
    :param led: Led对象
    """
    try:
        while True:
            # 亮度逐渐增加
            for level in range(0, 10):
                led.set_brightness(level)
                time.sleep(0.3)  # 每个等级保持0.3秒

            # 亮度保持在最高等级一段时间
            time.sleep(1)

            # 亮度逐渐减少
            for level in range(8, -1, -1):
                led.set_brightness(level)
                time.sleep(0.3)  # 每个等级保持0.3秒

            # 亮度保持在最低等级一段时间
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序中断，退出...")
    finally:
        led.cleanup()


if __name__ == "__main__":
    led = Led(GPIO_LED, FREQUENCY)
    trapezoidal_brightness_change(led)
