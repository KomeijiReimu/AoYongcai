import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO_PHASE_A = 32
GPIO_PHASE_B = 36
GPIO_PHASE_C = 38
GPIO_PHASE_D = 40


class Stepping_motor:
    def __init__(self, pin):
        # pin 为 (GPIO_PHASE_A, GPIO_PHASE_B, GPIO_PHASE_C, GPIO_PHASE_D)
        self.pulse = [
            (1, 0, 0, 0),
            (1, 1, 0, 0),
            (0, 1, 0, 0),
            (0, 1, 1, 0),
            (0, 0, 1, 0),
            (0, 0, 1, 1),
            (0, 0, 0, 1),
            (1, 0, 0, 1)
        ]
        self.phaseA, self.phaseB, self.phaseC, self.phaseD = pin
        GPIO.setup(self.phaseA, GPIO.OUT)
        GPIO.setup(self.phaseB, GPIO.OUT)
        GPIO.setup(self.phaseC, GPIO.OUT)
        GPIO.setup(self.phaseD, GPIO.OUT)

        # 初始化所有相位为低电平
        GPIO.output(self.phaseA, GPIO.LOW)
        GPIO.output(self.phaseB, GPIO.LOW)
        GPIO.output(self.phaseC, GPIO.LOW)
        GPIO.output(self.phaseD, GPIO.LOW)

    def __set_step(self, pulseIn):
        p1, p2, p3, p4 = pulseIn
        GPIO.output(self.phaseA, p1)
        GPIO.output(self.phaseB, p2)
        GPIO.output(self.phaseC, p3)
        GPIO.output(self.phaseD, p4)
        print(f"Set steps: A={p1}, B={p2}, C={p3}, D={p4}")  # 调试信息
        time.sleep(0.001)  # 增加延迟，确保电机能正确响应

    def degrees_to_steps(self, degrees):
        """
        根据旋转角度计算所需的步数（脉冲数）。

        :param degrees: 需要旋转的角度
        :return: 所需的步数（脉冲数）
        """
        pulses_per_revolution = 4096  # 输出轴转一圈所需脉冲数
        steps = int(pulses_per_revolution * degrees / 360)
        return steps

    def rotate(self, direction=0, degrees=90):
        """
        旋转步进电机指定的角度。

        :param direction: 旋转方向，0为顺时针，1为逆时针
        :param degrees: 需要旋转的角度
        """
        steps = self.degrees_to_steps(degrees)
        delay = 0.001  # 延迟时间，单位为秒
        print(f"Rotating {degrees} degrees {'顺时针' if direction == 0 else '逆时针'} ({steps} 步)")

        for step in range(steps):
            if direction == 0:  # 顺时针
                pulse = self.pulse[step % len(self.pulse)]
            else:  # 逆时针
                pulse = self.pulse[::-1][step % len(self.pulse)]
            self.__set_step(pulse)
            time.sleep(delay)

        # 确保旋转完成后关闭所有引脚
        GPIO.output(self.phaseA, GPIO.LOW)
        GPIO.output(self.phaseB, GPIO.LOW)
        GPIO.output(self.phaseC, GPIO.LOW)
        GPIO.output(self.phaseD, GPIO.LOW)

    def clean(self):
        GPIO.cleanup()
        print("GPIO cleaned up.")


if __name__ == '__main__':
    try:
        motor = Stepping_motor((GPIO_PHASE_A, GPIO_PHASE_B, GPIO_PHASE_C, GPIO_PHASE_D))
        while True:
            direction_input = input("输入方向: 0 顺时针 1 逆时针 (输入 'exit' 退出): ")
            if direction_input.lower() == 'exit':
                break
            if direction_input not in ['0', '1']:
                print("无效输入，请输入 0 或 1。")
                continue
            direction = int(direction_input)
            motor.rotate(direction=direction, degrees=90)  # 每次旋转90度
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        motor.clean()
