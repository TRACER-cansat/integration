#LED統合
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
# グローバル設定・初期化
# ==============================================================================

# GPIO番号はBCM方式で指定
GPIO.setmode(GPIO.BCM)

# ------------------------------------------------------------------------------
# 超音波センサーの設定
# ------------------------------------------------------------------------------
SENSORS = [
    ("Sensor 1", 23, 24),  # 名前、TRIG、ECHO
    ("Sensor 2", 25, 26),
]

# ------------------------------------------------------------------------------
# LEDの設定
# ------------------------------------------------------------------------------
LED_PIN = 17  # BCM17 = 物理ピン11
led_state = False

# ------------------------------------------------------------------------------
# BNO055の設定
# ------------------------------------------------------------------------------
i2c = board.I2C()
bno_sensor = adafruit_bno055.BNO055_I2C(i2c)

# BNO055の速度算出用変数
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0
last_time = time.time()

# ------------------------------------------------------------------------------
# キーボード入力用変数
# ------------------------------------------------------------------------------
old_terminal_settings = None
keyboard_enabled = False


# ==============================================================================
# [1] 超音波センサー関連の関数
# ==============================================================================

def init_ultrasonic():
    """超音波センサーのGPIOピンを初期設定する"""
    for name, trig, echo in SENSORS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, GPIO.LOW)

    # センサーが安定するまで少し待つ
    time.sleep(0.1)


def distance(trig, echo, timeout=0.03):
    """
    個々の超音波センサーから距離を取得する。

    戻り値:
        測定成功: 距離 [cm]
        測定失敗: None
    """

    # TRIGを一度LOWにする
    GPIO.output(trig, GPIO.LOW)
    time.sleep(0.000002)

    # 10マイクロ秒のパルスを送信
    GPIO.output(trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig, GPIO.LOW)

    # ECHOがHIGHになるまで待つ
    start_wait = time.monotonic()

    while GPIO.input(echo) == GPIO.LOW:
        if time.monotonic() - start_wait > timeout:
            return None

    pulse_start = time.monotonic()

    # ECHOがLOWに戻るまで待つ
    while GPIO.input(echo) == GPIO.HIGH:
        if time.monotonic() - pulse_start > timeout:
            return None

    pulse_end = time.monotonic()

    # 超音波の往復時間
    pulse_duration = pulse_end - pulse_start

    # 距離 = 往復時間 × 音速 ÷ 2
    distance_cm = (pulse_duration * 34300) / 2

    return distance_cm


def measure_ultrasonic(csv_writer):
    """2台の超音波センサーを測定し、CSVへ保存する"""

    current_data = {
        "Sensor 1": {
            "time": None,
            "distance": "timeout"
        },
        "Sensor 2": {
            "time": None,
            "distance": "timeout"
        }
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

        # 2台の超音波センサーの相互干渉を防ぐ
        time.sleep(0.06)

    # CSVファイルへ書き込む
    csv_writer.writerow([
        current_data["Sensor 1"]["time"],
        current_data["Sensor 1"]["distance"],
        current_data["Sensor 2"]["time"],
        current_data["Sensor 2"]["distance"]
    ])

    print("-" * 40)


# ==============================================================================
# [2] BNO055 9軸センサー関連の関数
# ==============================================================================

def get_compass_direction(heading):
    """方位角0～360度を16方位の文字列へ変換する"""

    if heading is None:
        return "---"

    directions = [
        "北",
        "北北東",
        "北東",
        "東北東",
        "東",
        "東南東",
        "南東",
        "南南東",
        "南",
        "南南西",
        "南西",
        "西南西",
        "西",
        "西北西",
        "北西",
        "北北西"
    ]

    index = int((heading + 11.25) / 22.5) % 16

    return directions[index]


def measure_bno055(csv_writer):
    """BNO055の測定、速度計算、表示、CSV書き込みを行う"""

    global velocity_x
    global velocity_y
    global velocity_z
    global last_time

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # BNO055からデータを取得
    euler = bno_sensor.euler
    linear_acceleration = bno_sensor.linear_acceleration
    gyro = bno_sensor.gyro
    calibration = bno_sensor.calibration_status

    # 初期値
    heading = None
    roll = None
    pitch = None

    ax = None
    ay = None
    az = None

    # --------------------------------------------------------------------------
    # 加速度と速度の計算
    # --------------------------------------------------------------------------
    if (
        linear_acceleration
        and linear_acceleration[0] is not None
        and linear_acceleration[1] is not None
        and linear_acceleration[2] is not None
    ):
        # 小さい測定ノイズを0として扱う
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

        # 速度 = 前回の速度 + 加速度 × 経過時間
        velocity_x += ax * dt
        velocity_y += ay * dt
        velocity_z += az * dt

    # --------------------------------------------------------------------------
    # 姿勢角の取得
    # --------------------------------------------------------------------------
    if (
        euler
        and euler[0] is not None
        and euler[1] is not None
        and euler[2] is not None
    ):
        heading = euler[0]
        roll = euler[1]
        pitch = euler[2]

    # --------------------------------------------------------------------------
    # 画面表示
    # --------------------------------------------------------------------------
    print("\033[2J\033[H", end="")

    print("--- Integrated sensor realtime data ---")
    print(f"【 LED状態 】 {'ON' if led_state else 'OFF'}")
    print("【 LED操作 】 任意のキーを押すとON/OFF切り替え")
    print("【 終了方法 】 Ctrl+C")
    print("-" * 60)

    if heading is not None:
        direction_string = get_compass_direction(heading)

        print(
            f"【 傾 き 】"
            f" Roll(左右): {roll:6.1f}° ,"
            f" Pitch(前後): {pitch:6.1f}°"
        )

        print(
            f"【 方 位 】"
            f" Heading: {heading:6.1f}°"
            f" ({direction_string}方向)"
        )
    else:
        print("【 傾 き 】 取得中...")
        print("【 方 位 】 取得中...")

    if ax is not None:
        print(
            f"【加速度】"
            f" X: {ax:6.2f} m/s²,"
            f" Y: {ay:6.2f} m/s²,"
            f" Z: {az:6.2f} m/s²"
        )

        print(
            f"【 速 度 】"
            f" X: {velocity_x:6.2f} m/s,"
            f" Y: {velocity_y:6.2f} m/s,"
            f" Z: {velocity_z:6.2f} m/s"
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
            f"【角速度】"
            f" X: {gyro[0]:6.2f} rad/s,"
            f" Y: {gyro[1]:6.2f} rad/s,"
            f" Z: {gyro[2]:6.2f} rad/s"
        )
    else:
        print("【角速度】 取得中...")

    if calibration and len(calibration) == 4:
        print(
            "【校正状態】"
            f" Sys: {calibration[0]},"
            f" Gyro: {calibration[1]},"
            f" Accel: {calibration[2]},"
            f" Mag: {calibration[3]}"
        )
    else:
        print("【校正状態】 取得中...")

    # --------------------------------------------------------------------------
    # CSVファイルへの書き込み
    # --------------------------------------------------------------------------
    csv_writer.writerow([
        current_time,
        heading,
        roll,
        pitch,
        velocity_x,
        velocity_y,
        velocity_z
    ])


# ==============================================================================
# [3] LED関連の関数
# ==============================================================================

def init_led():
    """LEDのGPIOピンを初期設定する"""

    global led_state

    GPIO.setup(LED_PIN, GPIO.OUT)

    led_state = False
    GPIO.output(LED_PIN, GPIO.LOW)


def toggle_led():
    """LEDのON/OFF状態を切り替える"""

    global led_state

    led_state = not led_state

    if led_state:
        GPIO.output(LED_PIN, GPIO.HIGH)
        print("LED: ON")
    else:
        GPIO.output(LED_PIN, GPIO.LOW)
        print("LED: OFF")


def close_led():
    """プログラム終了時にLEDを消灯する"""

    global led_state

    led_state = False
    GPIO.output(LED_PIN, GPIO.LOW)


# ==============================================================================
# [4] キーボード入力関連の関数
# ==============================================================================

def init_keyboard():
    """
    Enterを押さなくてもキー入力を検出できるようにする。

    ターミナル以外から実行した場合は、
    キー操作を無効にしてセンサー測定だけを続ける。
    """

    global old_terminal_settings
    global keyboard_enabled

    if not sys.stdin.isatty():
        print("警告: ターミナル入力を使用できません。")
        print("LEDのキー操作は無効になります。")
        keyboard_enabled = False
        return

    old_terminal_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    keyboard_enabled = True


def check_keyboard():
    """
    キーが押されているかを確認する。

    キーが押されていない場合もプログラムを停止させない。
    """

    if not keyboard_enabled:
        return

    # 入力可能なキーがある場合だけ読み取る
    readable, _, _ = select.select(
        [sys.stdin],
        [],
        [],
        0
    )

    if readable:
        key = sys.stdin.read(1)

        # 環境によってCtrl+Cが文字として取得された場合
        if key == "\x03":
            raise KeyboardInterrupt

        toggle_led()


def wait_with_keyboard(duration):
    """
    指定時間待機しながらキー入力を監視する。

    time.sleep(duration)だけを使う場合と異なり、
    待機中もLEDを操作できる。
    """

    end_time = time.monotonic() + duration

    while time.monotonic() < end_time:
        check_keyboard()
        time.sleep(0.05)


def restore_keyboard():
    """終了時にターミナルの入力設定を元へ戻す"""

    if keyboard_enabled and old_terminal_settings is not None:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings
        )


# ==============================================================================
# メイン実行処理
# ==============================================================================

def main():
    """センサー測定とLED操作を繰り返すメイン関数"""

    f_dist = None
    f_bno = None

    print("Measurement started...")
    print("任意のキー: LEDのON/OFF")
    print("Ctrl+C: プログラム終了")
    print("-" * 60)

    try:
        # ----------------------------------------------------------------------
        # 各装置の初期化
        # ----------------------------------------------------------------------
        init_ultrasonic()
        init_led()
        init_keyboard()

        # ----------------------------------------------------------------------
        # CSVファイルを開く
        # ----------------------------------------------------------------------
        f_dist = open(
            "distance_log.csv",
            "w",
            newline="",
            encoding="utf-8"
        )

        f_bno = open(
            "bno055_data.csv",
            "w",
            newline="",
            encoding="utf-8"
        )

        writer_dist = csv.writer(f_dist)
        writer_bno = csv.writer(f_bno)

        # ----------------------------------------------------------------------
        # CSVヘッダー
        # ----------------------------------------------------------------------
        writer_dist.writerow([
            "S1_Timestamp",
            "S1_Distance_cm",
            "S2_Timestamp",
            "S2_Distance_cm"
        ])

        writer_bno.writerow([
            "timestamp",
            "heading",
            "roll",
            "pitch",
            "velocity_x",
            "velocity_y",
            "velocity_z"
        ])

        time.sleep(0.5)

        # ----------------------------------------------------------------------
        # 繰り返し測定
        # ----------------------------------------------------------------------
        while True:
            # LED操作の確認
            check_keyboard()

            # BNO055の測定
            measure_bno055(writer_bno)

            # 超音波センサーの測定
            measure_ultrasonic(writer_dist)

            # CSVデータを即時反映
            f_dist.flush()
            f_bno.flush()

            # 1秒待機しながらLEDキー入力を監視
            wait_with_keyboard(1.0)

    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")

    except Exception as error:
        print(f"\nエラーが発生しました: {error}")

    finally:
        # ----------------------------------------------------------------------
        # 終了処理
        # ----------------------------------------------------------------------
        try:
            close_led()
        except Exception:
            pass

        restore_keyboard()

        if f_dist is not None:
            f_dist.close()

        if f_bno is not None:
            f_bno.close()

        GPIO.cleanup()

        print("LEDを消灯しました。")
        print("GPIOとCSVファイルを終了しました。")


if __name__ == "__main__":
    main()








#コードの流れ
#プログラム開始
#    ↓
#超音波センサー初期化
#    ↓
#LED初期化・消灯
#    ↓
#BNO055初期化
#    ↓
#キーボード入力設定
#    ↓
#BNO055を測定
#    ↓
#超音波センサー2台を測定
#    ↓
#CSVへ保存
#    ↓
#待機しながらキー入力を確認
#    ↓
#キーが押されたらLEDを切り替える
#    ↓
#繰り返し
