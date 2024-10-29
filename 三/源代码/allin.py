import RPi.GPIO as GPIO
import time
import threading
import queue

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# 定义GPIO引脚
GPIO_BUZZER = 29  # 蜂鸣器引脚
GPIO_LED = 7  # LED引脚
GPIO_PHASE_A = 32
GPIO_PHASE_B = 36
GPIO_PHASE_C = 38
GPIO_PHASE_D = 40

# 全局停止事件，用于停止所有线程
stop_event = threading.Event()

motor_command_queue = queue.Queue()


# 蜂鸣器类
class Buzzer(threading.Thread):
    def __init__(self, buzzer_pin, on_time=0.5, off_time=0.5):
        super().__init__()
        self.buzzer_pin = buzzer_pin
        self.on_time = on_time
        self.off_time = off_time
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        GPIO.output(self.buzzer_pin, GPIO.LOW)

    def run(self):
        count = 0
        while not stop_event.is_set():
            count += 1
            print(f"[Buzzer] 蜂鸣器叫唤第 {count} 次")
            GPIO.output(self.buzzer_pin, GPIO.HIGH)
            print(f"[Buzzer] GPIO 引脚 {self.buzzer_pin} 输出高电平，蜂鸣器开启")
            time.sleep(self.on_time)
            GPIO.output(self.buzzer_pin, GPIO.LOW)
            print(f"[Buzzer] GPIO 引脚 {self.buzzer_pin} 输出低电平，蜂鸣器关闭")
            time.sleep(self.off_time)
        print("[Buzzer] 停止蜂鸣器线程")


# LED类
class Led(threading.Thread):
    def __init__(self, pin, frequency=1000):
        super().__init__()
        self.pin = pin
        self.frequency = frequency
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
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
        print(f"[LED] 设置亮度等级: {level}, 占空比: {duty_cycle:.2f}%")

    def run(self):
        try:
            while not stop_event.is_set():
                # 亮度逐渐增加
                for level in range(0, 10):
                    if stop_event.is_set():
                        break
                    self.set_brightness(level)
                    time.sleep(0.3)  # 每个等级保持0.3秒

                if stop_event.is_set():
                    break

                # 亮度保持在最高等级一段时间
                print("[LED] 亮度保持在最高等级")
                time.sleep(1)

                # 亮度逐渐减少
                for level in range(8, -1, -1):
                    if stop_event.is_set():
                        break
                    self.set_brightness(level)
                    time.sleep(0.3)  # 每个等级保持0.3秒

                if stop_event.is_set():
                    break

                # 亮度保持在最低等级一段时间
                print("[LED] 亮度保持在最低等级")
                time.sleep(1)
        except Exception as e:
            print(f"[LED] 发生错误: {e}")
        finally:
            self.cleanup()
            print("[LED] 停止LED线程")

    def cleanup(self):
        self.pwm.stop()
        GPIO.output(self.pin, GPIO.LOW)


# 步进电机类
class SteppingMotor(threading.Thread):
    def __init__(self, pins, rotation_delay=0.001):
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
        # 设置引脚为输出并初始化为低电平
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    def degrees_to_steps(self, degrees):
        """
        根据旋转角度计算所需的步数（脉冲数）。
        :param degrees: 需要旋转的角度
        :return: 所需的步数（脉冲数）
        """
        pulses_per_revolution = 4096  # 输出轴转一圈所需脉冲数
        steps = int(pulses_per_revolution * degrees / 360)
        return steps

    def set_step(self, pulse):
        p1, p2, p3, p4 = pulse
        GPIO.output(self.pins[0], p1)
        GPIO.output(self.pins[1], p2)
        GPIO.output(self.pins[2], p3)
        GPIO.output(self.pins[3], p4)
        print(f"[Motor] 设置步骤: A={p1}, B={p2}, C={p3}, D={p4}")
        time.sleep(self.rotation_delay)  # 增加延迟，确保电机能正确响应

    def rotate_motor(self, direction=0, degrees=90):
        """
        旋转步进电机指定的角度。
        :param direction: 旋转方向，0为顺时针，1为逆时针
        :param degrees: 需要旋转的角度
        """
        steps = self.degrees_to_steps(degrees)
        print(f"[Motor] 旋转 {degrees} 度 {'顺时针' if direction == 0 else '逆时针'} ({steps} 步)")
        for step in range(steps):
            if stop_event.is_set():
                break
            if direction == 0:  # 顺时针
                pulse = self.pulse_sequence[step % len(self.pulse_sequence)]
            else:  # 逆时针
                pulse = self.pulse_sequence[::-1][step % len(self.pulse_sequence)]
            self.set_step(pulse)
        # 确保旋转完成后关闭所有引脚
        GPIO.output(self.pins[0], GPIO.LOW)
        GPIO.output(self.pins[1], GPIO.LOW)
        GPIO.output(self.pins[2], GPIO.LOW)
        GPIO.output(self.pins[3], GPIO.LOW)
        print("[Motor] 旋转完成，关闭所有引脚")

    def run(self):
        try:
            while not stop_event.is_set():
                try:
                    # 等待旋转命令，超时后继续检查是否停止
                    command = motor_command_queue.get(timeout=0.5)
                    if command['action'] == 'open':
                        self.rotate_motor(direction=1, degrees=90)  # 逆时针旋转90°打开
                    elif command['action'] == 'close':
                        self.rotate_motor(direction=0, degrees=90)  # 顺时针旋转90°关闭
                except queue.Empty:
                    continue
        except Exception as e:
            print(f"[Motor] 发生错误: {e}")
        finally:
            self.cleanup()
            print("[Motor] 停止步进电机线程")

    def cleanup(self):
        GPIO.output(self.pins[0], GPIO.LOW)
        GPIO.output(self.pins[1], GPIO.LOW)
        GPIO.output(self.pins[2], GPIO.LOW)
        GPIO.output(self.pins[3], GPIO.LOW)


# 主程序
def main():
    # 创建各个设备的实例
    buzzer = Buzzer(GPIO_BUZZER, on_time=0.5, off_time=0.5)
    led = Led(GPIO_LED, frequency=1000)
    motor = SteppingMotor((GPIO_PHASE_A, GPIO_PHASE_B, GPIO_PHASE_C, GPIO_PHASE_D),
                          rotation_delay=0.001)

    # 启动线程
    buzzer.start()
    led.start()
    motor.start()

    try:
        while not stop_event.is_set():
            print("\n请输入命令：")
            print("o - 打开水龙头")
            print("c - 关闭水龙头")
            print("x - 退出程序")
            user_input = input("命令: ").strip().lower()

            if user_input == 'o':
                print("[Main] 接收到打开水龙头的命令")
                motor_command_queue.put({'action': 'open'})
            elif user_input == 'c':
                print("[Main] 接收到关闭水龙头的命令")
                motor_command_queue.put({'action': 'close'})
            elif user_input == 'x':
                print("[Main] 接收到退出命令")
                stop_event.set()
            else:
                print("[Main] 无效命令，请输入 'o', 'c' 或 'x'")
    except KeyboardInterrupt:
        print("\n[Main] 程序中断，准备退出...")
        stop_event.set()
    finally:
        # 等待所有线程完成
        buzzer.join()
        led.join()
        motor.join()
        GPIO.cleanup()
        print("[Main] GPIO 设置已清理，程序退出。")


if __name__ == "__main__":
    main()
