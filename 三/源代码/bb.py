import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO_BUZZER = 29  # 连接蜂鸣器的引脚

GPIO.setup(GPIO_BUZZER, GPIO.OUT)
GPIO.output(GPIO_BUZZER, GPIO.LOW)


def beep(buzzer_pin, on_time=0.5, off_time=0.5, repetitions=None):
    """
    控制蜂鸣器周期性叫唤。

    :param buzzer_pin: 连接蜂鸣器的GPIO引脚号
    :param on_time: 蜂鸣器开启的时间（秒）
    :param off_time: 蜂鸣器关闭的时间（秒）
    :param repetitions: 叫唤次数，默认为None表示无限循环
    """
    try:
        count = 0
        while True:
            if repetitions and count >= repetitions:
                break
            count += 1
            print(f"蜂鸣器叫唤第 {count} 次")
            GPIO.output(buzzer_pin, GPIO.HIGH)
            print(f"GPIO 引脚 {buzzer_pin} 输出高电平，蜂鸣器开启")
            time.sleep(on_time)
            GPIO.output(buzzer_pin, GPIO.LOW)
            print(f"GPIO 引脚 {buzzer_pin} 输出低电平，蜂鸣器关闭")
            time.sleep(off_time)
    except KeyboardInterrupt:
        print("\n程序中断，退出...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        print("清理 GPIO 设置")
        GPIO.cleanup()


if __name__ == "__main__":
    beep(GPIO_BUZZER, on_time=0.5, off_time=0.5, repetitions=None)
