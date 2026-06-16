import RPi.GPIO as GPIO
import time
import csv
import board
import adafruit_bno055

# ==============================================================================
# グローバル設定・初期化
# ==============================================================================
# GPIOの設定
GPIO.setmode(GPIO.BCM)
SENSORS = [
    ("Sensor 1", 23, 24),
    ("Sensor 2", 25, 26),
]

# BNO055の設定
i2c = board.I2C()
bno_sensor = adafruit_bno055.BNO055_I2C(i2c)

# BNO055の速度算出用グローバル変数
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0
last_time = time.time()

# ==============================================================================
# [1] 超音波センサー関連の関数
# ==============================================================================
def init_ultrasonic():
    """超音波センサーのGPIOピン初期設定"""
    for name, trig, echo in SENSORS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, False)

def distance(trig, echo, timeout=0.03):
    """個々の超音波センサーから距離を取得する"""
    GPIO.output(trig, False)
    time.sleep(0.000002)

    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_wait = time.monotonic()
    while GPIO.input(echo) == 0:
        if time.monotonic() - start_wait > timeout:
            return None

    pulse_start = time.monotonic()

    while GPIO.input(echo) == 1:
        if time.monotonic() - pulse_start > timeout:
            return None

    pulse_end = time.monotonic()

    pulse_duration = pulse_end - pulse_start
    return (pulse_duration * 34300) / 2

def measure_ultrasonic(csv_writer):
    """超音波センサー全体の計測・表示・データ返却"""
    current_data = {
        "Sensor 1": {"time": None, "distance": "timeout"},
        "Sensor 2": {"time": None, "distance": "timeout"}
    }

    for name, trig, echo in SENSORS:
        t = time.time()
        dist = distance(trig, echo)

        current_data[name]["time"] = t
        if dist is None:
            print(f"{name}: timeout")
        else:
            print(f"{name}: {dist:.2f} cm")
            current_data[name]["distance"] = dist
        time.sleep(0.06)

    # CSVファイルにデータを書き込む
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
    """方位角(0-360)から16方位の文字列を返す関数"""
    if heading is None: return "---"
    directions = ["北", "北北東", "北東", "東北東", "東", "東南東", "南東", "南南東", 
                  "南", "南南西", "南西", "西南西", "西", "西北西", "北西", "北北西"]
    idx = int((heading + 11.25) / 22.5) % 16
    return directions[idx]

def measure_bno055(csv_writer):
    """BNO055の計測・計算・表示・CSV書き込み"""
    global velocity_x, velocity_y, velocity_z, last_time
    
    current_time = time.time()
    dt = current_time - last_time  # 前回の計測からの経過時間 (秒)
    last_time = current_time

    euler = bno_sensor.euler
    lin_accel = bno_sensor.linear_acceleration 
    gyro = bno_sensor.gyro
    calib = bno_sensor.calibration_status

    # 画面クリア
    print("\033[H\033[j", end="")
    print("--- BNO055 realtime data & calculation ---")

    # 1. 水平に対する傾き(Roll/Pitch) と 2. 方位(Heading)
    if euler and euler[0] is not None:
        heading = euler[0]
        roll = euler[1]
        pitch = euler[2]
        direction_str = get_compass_direction(heading)

        # CSVファイルにデータを書き込む
        csv_writer.writerow([time.time(), heading, roll, pitch, velocity_x, velocity_y, velocity_z])

        print(f"【 傾 き 】 Roll(左右): {roll:6.1f}° , Pitch(前後): {pitch:6.1f}° (水平=0°)")
        print(f"【 方 位 】 Heading: {heading:6.1f}° ({direction_str}方向)")
    else:
        print("【 傾 き 】 取得中...")
        print("【 方 位 】 取得中...")

    # 3. 加速度と速度の算出
    if lin_accel and lin_accel[0] is not None:
        threshold = 0.1 
        ax = lin_accel[0] if abs(lin_accel[0]) > threshold else 0.0
        ay = lin_accel[1] if abs(lin_accel[1]) > threshold else 0.0
        az = lin_accel[2] if abs(lin_accel[2]) > threshold else 0.0

        # 速度 = 速度 + (加速度 * 経過時間)
        velocity_x += ax * dt
        velocity_y += ay * dt
        velocity_z += az * dt

        print(f"【加速度】 X: {ax:6.2f} m/s², Y: {ay:6.2f} m/s², Z: {az:6.2f} m/s² (重力除去済)")
        print(f"【 速 度 】 X: {velocity_x:6.2f} m/s , Y: {velocity_y:6.2f} m/s , Z: {velocity_z:6.2f} m/s")
    else:
        print("【加速度】 取得中...")
        print("【 速 度 】 取得中...")

    # その他の参考データ
    if gyro and gyro[0] is not None:
        print(f"Gyroscope   - X: {gyro[0]:6.2f} /s, Y: {gyro[1]:6.2f} /s, Z: {gyro[2]:6.2f} /s")
    
    print(f"Calibration - Sys: {calib[0]}, Gyro: {calib[1]}, Accel: {calib[2]}, Mag: {calib[3]}")

# ==============================================================================
# メイン実行処理
# ==============================================================================
def main():
    print("Measurement started... Press Ctrl+C to stop.")
    print("-" * 60)
    
    # センサー初期化
    init_ultrasonic()

    # それぞれのCSVファイルを開く
    f_dist = open("distance_log.csv", "w", newline="")
    f_bno = open("bno055_data.csv", "w", newline="")
    
    writer_dist = csv.writer(f_dist)
    writer_bno = csv.writer(f_bno)
    
    # ヘッダー書き込み
    writer_dist.writerow(["S1_Timestamp", "S1_Distance_cm", "S2_Timestamp", "S2_Distance_cm"])
    writer_bno.writerow(['timestamp', 'heading', 'roll', 'pitch', 'velocity_x', 'velocity_y', 'velocity_z'])

    try:
        time.sleep(0.5)

        while True:
            # 各センサーの計測関数を順に呼び出し
            measure_bno055(writer_bno)
            measure_ultrasonic(writer_dist)

            # ディスクへの即時書き込みを保証
            f_dist.flush()
            f_bno.flush()

            # 元コード[1]の末尾の待機時間
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nMeasurement stopped by User")

    finally:
        # 終了処理
        GPIO.cleanup()
        f_dist.close()
        f_bno.close()

if __name__ == "__main__":
    main()