import RPi.GPIO as GPIO
import time
import csv
import board
import adafruit_bno055
import sys
import termios
import tty
import select


# ==============================================================================
# グローバル設定
# ==============================================================================
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# 超音波センサー: 名前、TRIG、ECHO
SENSORS = [
    ("Sensor 1", 23, 24),
    ("Sensor 2", 25, 26),
]

# 連続回転サーボ
SERVO1_PIN = 13
SERVO2_PIN = 18
PWM_FREQ = 50
STOP = 7.3
FORWARD = 10.0

# サーボごとに低速値を個別設定
# Servo 1は8.5%では速度差が出にくいため、停止値7.3%に近い7.8%を使用
SERVO1_SLOW = 7.8

# Servo 2は現在の8.5%で速度差が確認できているため、そのまま使用
SERVO2_SLOW = 8.4

REVERSE = 5.0

servo1 = None
servo2 = None
servo_command = "STOP"
servo1_duty = STOP
servo2_requested_duty = STOP
servo2_output_duty = STOP

# BNO055
BNO_ADDRESS = 0x28
BNO_REINIT_INTERVAL = 2.0

# I2Cバスはプログラム中に1回だけ生成して使い回す
# 再接続のたびにboard.I2C()を作り直さない
bno_i2c = None
bno_sensor = None
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0
last_measurement_time = None
last_bno_reinit_attempt = 0.0
bno_status = "未初期化"

# 測定周期
# キー操作の有無に関係なく、この周期で自動測定する
BNO_MEASUREMENT_INTERVAL = 0.2          # BNO055: 5 Hz
ULTRASONIC_MEASUREMENT_INTERVAL = 1.0  # 超音波: 1 Hz

# キーボード入力
old_terminal_settings = None
keyboard_enabled = False


# ==============================================================================
# 超音波センサー関連
# ==============================================================================
def init_ultrasonic():
    for _name, trig, echo in SENSORS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, GPIO.LOW)
    time.sleep(0.1)


def distance(trig, echo, timeout=0.03):
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    wait_start = time.monotonic()
    while GPIO.input(echo) == GPIO.LOW:
        if time.monotonic() - wait_start > timeout:
            return None

    pulse_start = time.monotonic()
    while GPIO.input(echo) == GPIO.HIGH:
        if time.monotonic() - pulse_start > timeout:
            return None

    pulse_end = time.monotonic()
    return (pulse_end - pulse_start) * 34300 / 2


def measure_ultrasonic(csv_writer):
    current_data = {
        "Sensor 1": {"time": None, "distance": "timeout"},
        "Sensor 2": {"time": None, "distance": "timeout"},
    }

    for name, trig, echo in SENSORS:
        measurement_time = time.time()
        dist = distance(trig, echo)
        current_data[name]["time"] = measurement_time

        if dist is None:
            print(f"{name}: timeout")
        else:
            current_data[name]["distance"] = dist
            print(f"{name}: {dist:.2f} cm")

        # 2台の超音波センサーの干渉を減らす
        time.sleep(0.06)

    csv_writer.writerow([
        current_data["Sensor 1"]["time"],
        current_data["Sensor 1"]["distance"],
        current_data["Sensor 2"]["time"],
        current_data["Sensor 2"]["distance"],
    ])
    print("-" * 60)


# ==============================================================================
# BNO055関連
# ==============================================================================
def init_bno055(retries=5):
    """BNO055を初期化する。I2Cバスは1回だけ作成して使い回す。"""
    global bno_i2c
    global bno_sensor
    global last_measurement_time
    global bno_status

    last_error = None

    # I2Cバスは初回だけ生成する。
    # 通信エラー後の再接続でも同じバスを使い回す。
    if bno_i2c is None:
        try:
            bno_i2c = board.I2C()
        except (OSError, RuntimeError, ValueError) as error:
            bno_status = f"I2C初期化失敗: {error}"
            print(f"I2C bus initialization failed: {error}")
            return False

        # BNO055の電源投入・リセット完了を待つ
        time.sleep(1.0)

    for attempt in range(1, retries + 1):
        try:
            sensor = adafruit_bno055.BNO055_I2C(
                bno_i2c,
                address=BNO_ADDRESS,
            )

            # 動作モードへの切り替え後、最初の測定を少し待つ
            time.sleep(0.7)

            bno_sensor = sensor
            last_measurement_time = time.monotonic()
            bno_status = "接続中"
            print("BNO055 initialization successful.")
            return True

        except (OSError, RuntimeError, ValueError) as error:
            last_error = error
            bno_sensor = None
            last_measurement_time = None
            bno_status = f"初期化失敗: {error}"

            print(
                f"BNO055 initialization failed "
                f"({attempt}/{retries}): {error}"
            )

            if attempt < retries:
                time.sleep(1.0)

    print(f"BNO055を初期化できませんでした: {last_error}")
    return False


def reinitialize_bno055_if_due():
    """通信切断後、一定間隔でBNO055の再接続を試す。"""
    global last_bno_reinit_attempt
    global bno_status

    now = time.monotonic()
    if now - last_bno_reinit_attempt < BNO_REINIT_INTERVAL:
        return False

    last_bno_reinit_attempt = now
    bno_status = "再初期化中"
    print("BNO055を再初期化します...")
    return init_bno055(retries=1)


def get_compass_direction(heading):
    if heading is None:
        return "---"

    directions = [
        "北", "北北東", "北東", "東北東",
        "東", "東南東", "南東", "南南東",
        "南", "南南西", "南西", "西南西",
        "西", "西北西", "北西", "北北西",
    ]
    index = int((heading + 11.25) / 22.5) % 16
    return directions[index]


def measure_bno055(csv_writer):
    """BNO055を測定する。通信切断時もプログラムを終了しない。"""
    global bno_sensor
    global velocity_x
    global velocity_y
    global velocity_z
    global last_measurement_time
    global bno_status

    timestamp = time.time()
    heading = None
    roll = None
    pitch = None
    ax = None
    ay = None
    az = None
    gyro = None
    calibration = None
    communication_error = None

    if bno_sensor is None:
        reinitialize_bno055_if_due()

    if bno_sensor is not None:
        try:
            current_time = time.monotonic()
            if last_measurement_time is None:
                dt = 0.0
            else:
                dt = current_time - last_measurement_time
            last_measurement_time = current_time

            if dt < 0 or dt > 2.0:
                dt = 0.0

            euler = bno_sensor.euler
            linear_acceleration = bno_sensor.linear_acceleration
            gyro = bno_sensor.gyro
            calibration = bno_sensor.calibration_status

            if (
                euler
                and euler[0] is not None
                and euler[1] is not None
                and euler[2] is not None
            ):
                heading, roll, pitch = euler

            if (
                linear_acceleration
                and linear_acceleration[0] is not None
                and linear_acceleration[1] is not None
                and linear_acceleration[2] is not None
            ):
                threshold = 0.1
                ax = (
                    linear_acceleration[0]
                    if abs(linear_acceleration[0]) > threshold
                    else 0.0
                )
                ay = (
                    linear_acceleration[1]
                    if abs(linear_acceleration[1]) > threshold
                    else 0.0
                )
                az = (
                    linear_acceleration[2]
                    if abs(linear_acceleration[2]) > threshold
                    else 0.0
                )
                velocity_x += ax * dt
                velocity_y += ay * dt
                velocity_z += az * dt

            bno_status = "接続中"

        except (OSError, RuntimeError, ValueError) as error:
            communication_error = error
            bno_status = f"通信エラー: {error}"
            # 壊れたオブジェクトを次のループで使わない
            bno_sensor = None
            last_measurement_time = None

    # ターミナル表示
    print("\033[2J\033[H", end="")
    print("--- Integrated sensor and servo control ---")
    print(
        "w: 前進   a: Servo 1低速   d: Servo 2低速   "
        "s: 後退   space: 停止   q: 終了"
    )
    print("-" * 60)
    print(f"【サーボ動作】 {servo_command}")
    print(f"【Servo 1】指令デューティ比: {servo1_duty:.2f}%")
    print(
        f"【Servo 2】指令値: {servo2_requested_duty:.2f}% / "
        f"実出力: {servo2_output_duty:.2f}%"
    )
    print("-" * 60)
    print(f"【BNO055】 {bno_status}")

    if communication_error is not None:
        print(f" I2C通信エラーを検知: {communication_error}")
        print(" 次の測定時に再初期化を試します。")

    if heading is not None:
        direction = get_compass_direction(heading)
        print(f"【 傾 き 】 Roll: {roll:6.1f}°, Pitch: {pitch:6.1f}°")
        print(f"【 方 位 】 Heading: {heading:6.1f}° ({direction}方向)")
    else:
        print("【 傾 き 】 取得中...")
        print("【 方 位 】 取得中...")

    if ax is not None:
        print(
            f"【加速度】 X: {ax:6.2f} m/s², "
            f"Y: {ay:6.2f} m/s², Z: {az:6.2f} m/s²"
        )
        print(
            f"【 速 度 】 X: {velocity_x:6.2f} m/s, "
            f"Y: {velocity_y:6.2f} m/s, Z: {velocity_z:6.2f} m/s"
        )
    else:
        print("【加速度】 取得中...")
        print("【 速 度 】 取得中...")

    if (
        gyro
        and gyro[0] is not None
        and gyro[1] is not None
        and gyro[2] is not None
    ):
        print(
            f"【角速度】 X: {gyro[0]:6.2f} rad/s, "
            f"Y: {gyro[1]:6.2f} rad/s, Z: {gyro[2]:6.2f} rad/s"
        )
    else:
        print("【角速度】 取得中...")

    if calibration and len(calibration) == 4:
        print(
            f"【校正状態】 Sys: {calibration[0]}, "
            f"Gyro: {calibration[1]}, Accel: {calibration[2]}, "
            f"Mag: {calibration[3]}"
        )
    else:
        print("【校正状態】 取得中...")

    csv_writer.writerow([
        timestamp,
        heading,
        roll,
        pitch,
        ax,
        ay,
        az,
        velocity_x,
        velocity_y,
        velocity_z,
        servo_command,
        servo1_duty,
        servo2_requested_duty,
        servo2_output_duty,
        bno_status,
    ])


# ==============================================================================
# サーボ関連
# ==============================================================================
def init_servos():
    global servo1
    global servo2

    GPIO.setup(SERVO1_PIN, GPIO.OUT)
    GPIO.setup(SERVO2_PIN, GPIO.OUT)
    servo1 = GPIO.PWM(SERVO1_PIN, PWM_FREQ)
    servo2 = GPIO.PWM(SERVO2_PIN, PWM_FREQ)
    servo1.start(STOP)
    servo2.start(STOP)
    set_servo(STOP, STOP, "STOP")
    time.sleep(0.5)


def set_servo(s1, s2, command):
    global servo_command
    global servo1_duty
    global servo2_requested_duty
    global servo2_output_duty

    if servo1 is None or servo2 is None:
        return

    # Servo 2は取り付け方向が逆なのでSTOPを中心に反転する
    actual_servo2_duty = 2 * STOP - s2
    servo1.ChangeDutyCycle(s1)
    servo2.ChangeDutyCycle(actual_servo2_duty)

    servo_command = command
    servo1_duty = s1
    servo2_requested_duty = s2
    servo2_output_duty = actual_servo2_duty


def stop_servo_signal():
    """停止位置を送った後、PWMパルスも止める。"""
    global servo_command
    global servo1_duty
    global servo2_requested_duty
    global servo2_output_duty

    if servo1 is None or servo2 is None:
        return

    set_servo(STOP, STOP, "STOP")
    time.sleep(0.2)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)

    servo_command = "STOP (PWM OFF)"
    servo1_duty = 0.0
    servo2_requested_duty = 0.0
    servo2_output_duty = 0.0


def handle_servo_key(key):
    if key == "w":
        set_servo(FORWARD, FORWARD, "FORWARD")
    elif key == "a":
        set_servo(SERVO1_SLOW, FORWARD, "SERVO 1 SLOW")
    elif key == "d":
        set_servo(FORWARD, SERVO2_SLOW, "SERVO 2 SLOW")
    elif key == "s":
        set_servo(REVERSE, REVERSE, "REVERSE")
    elif key == " ":
        stop_servo_signal()
    elif key == "q":
        stop_servo_signal()
        return False
    return True


def close_servos():
    global servo1
    global servo2

    if servo1 is not None and servo2 is not None:
        stop_servo_signal()

    if servo1 is not None:
        servo1.stop()
        servo1 = None

    if servo2 is not None:
        servo2.stop()
        servo2 = None


def close_bno055():
    """終了時にBNO055参照とI2Cバスを安全に解放する。"""
    global bno_i2c
    global bno_sensor
    global bno_status

    bno_sensor = None
    bno_status = "終了"

    if bno_i2c is not None and hasattr(bno_i2c, "deinit"):
        try:
            bno_i2c.deinit()
        except Exception as error:
            print(f"I2C終了処理エラー: {error}")

    bno_i2c = None


# ==============================================================================
# キーボード入力関連
# ==============================================================================
def init_keyboard():
    global old_terminal_settings
    global keyboard_enabled

    if not sys.stdin.isatty():
        print("警告: ターミナルからのキー入力を使用できません。")
        keyboard_enabled = False
        return

    old_terminal_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    keyboard_enabled = True


def check_keyboard():
    if not keyboard_enabled:
        return True

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    if not readable:
        return True

    key = sys.stdin.read(1).lower()
    if key == "\x03":
        raise KeyboardInterrupt
    return handle_servo_key(key)


def wait_with_keyboard(duration):
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        if not check_keyboard():
            return False
        time.sleep(0.01)
    return True


def restore_keyboard():
    if keyboard_enabled and old_terminal_settings is not None:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )


# ==============================================================================
# メイン実行処理
# ==============================================================================
def main():
    f_dist = None
    f_bno = None

    try:
        print("Integrated measurement started.")
        print("w/a/d/s/space: サーボ操作")
        print("キー操作がなくてもセンサーデータを自動記録します。")
        print("q または Ctrl+C: 終了")
        print("-" * 60)

        init_ultrasonic()

        # サーボより先にBNO055を初期化し、起動時の電圧低下を避ける
        init_bno055(retries=5)

        init_servos()
        init_keyboard()

        f_dist = open(
            "distance_log.csv",
            "w",
            newline="",
            encoding="utf-8",
        )
        f_bno = open(
            "bno055_data.csv",
            "w",
            newline="",
            encoding="utf-8",
        )

        writer_dist = csv.writer(f_dist)
        writer_bno = csv.writer(f_bno)

        writer_dist.writerow([
            "S1_Timestamp",
            "S1_Distance_cm",
            "S2_Timestamp",
            "S2_Distance_cm",
        ])
        writer_bno.writerow([
            "timestamp",
            "heading",
            "roll",
            "pitch",
            "acceleration_x",
            "acceleration_y",
            "acceleration_z",
            "velocity_x",
            "velocity_y",
            "velocity_z",
            "servo_command",
            "servo1_duty",
            "servo2_requested_duty",
            "servo2_output_duty",
            "bno_status",
        ])

        time.sleep(0.5)
        running = True

        # キー入力とは独立した自動測定時刻
        next_bno_measurement = time.monotonic()
        next_ultrasonic_measurement = time.monotonic()

        while running:
            # キーが押されていなくても、ここでは停止しない
            running = check_keyboard()
            if not running:
                break

            now = time.monotonic()

            # BNO055は0.2秒ごとに自動測定
            if now >= next_bno_measurement:
                measure_bno055(writer_bno)
                f_bno.flush()

                # 処理が遅れた場合に測定を連続実行しないよう、
                # 現在時刻を基準に次回時刻を設定する
                next_bno_measurement = (
                    time.monotonic() + BNO_MEASUREMENT_INTERVAL
                )

            # 超音波センサーは1秒ごとに自動測定
            now = time.monotonic()
            if now >= next_ultrasonic_measurement:
                measure_ultrasonic(writer_dist)
                f_dist.flush()

                next_ultrasonic_measurement = (
                    time.monotonic()
                    + ULTRASONIC_MEASUREMENT_INTERVAL
                )

            # CPU使用率を抑えつつ、キー入力にはすぐ反応する
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nCtrl+Cが入力されました。")
    except Exception as error:
        print(f"\nエラーが発生しました: {error}")
    finally:
        try:
            close_servos()
        except Exception as error:
            print(f"サーボ終了処理エラー: {error}")

        restore_keyboard()

        if f_dist is not None:
            f_dist.close()
        if f_bno is not None:
            f_bno.close()

        close_bno055()
        GPIO.cleanup()
        print("サーボを停止しました。")
        print("GPIOとCSVファイルを終了しました。")


if __name__ == "__main__":
    main()
