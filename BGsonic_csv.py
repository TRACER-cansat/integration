# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import csv
import board
import adafruit_bno055
import DFRobot_GNSS  # 提供いただいたGPSライブラリ

# ==============================================================================
# グローバル設定・初期化
# ==============================================================================
# 1. GPIOの設定（超音波センサー用）
GPIO.setmode(GPIO.BCM)
SENSORS = [
    ("Sensor 1", 23, 24),
    ("Sensor 2", 25, 26),
]

# 2. BNO055の設定
i2c_bus = board.I2C()
bno_sensor = adafruit_bno055.BNO055_I2C(i2c_bus)

# BNO055の速度算出用グローバル変数
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0
last_time = time.time()

# 3. DFRobot GNSSの設定
GPS_BeiDou_GLONASS = DFRobot_GNSS.GPS_BeiDou_GLONASS
gnss = DFRobot_GNSS.DFRobot_GNSS_I2C(bus=1, addr=0x20)

# ==============================================================================
# 初期化関数
# ==============================================================================
def init_sensors():
    """すべてのセンサーの初期設定を行う"""
    # 超音波センサーのGPIO初期設定
    for name, trig, echo in SENSORS:
        GPIO.setup(trig, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        GPIO.output(trig, False)

    # GNSSの初期化と起動
    print("Initializing GNSS...")
    while not gnss.begin():
        print("Sensor initialize failed!! 请检查 I2C 接线、模块拨到 I2C、i2cdetect 是否能看到 0x20")
        time.sleep(1)
    print("GNSS initialize success!!")
    
    gnss.enable_power()
    gnss.set_gnss(GPS_BeiDou_GLONASS)
    gnss.rgb_on()

# ==============================================================================
# [1] 超音波センサー関連の関数
# ==============================================================================
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
    local_timestamp = time.time()  # 一番左の共通タイムスタンプ
    
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

    # 一番左に local_timestamp を追加してCSVに書き込み
    csv_writer.writerow([
        local_timestamp,
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
    
    current_time = time.time()  # 一番左の共通タイムスタンプ
    dt = current_time - last_time
    last_time = current_time

    euler = bno_sensor.euler
    lin_accel = bno_sensor.linear_acceleration 
    gyro = bno_sensor.gyro
    calib = bno_sensor.calibration_status

    # 画面クリア（Linuxターミナル用）
    print("\033[H\033[j", end="")
    print("--- Realtime Sensor Data Log ---")

    if euler and euler[0] is not None:
        heading = euler[0]
        roll = euler[1]
        pitch = euler[2]
        direction_str = get_compass_direction(heading)

        # 一番左に共通のタイムスタンプ(current_time)を追加してCSVに書き込む
        csv_writer.writerow([current_time, current_time, heading, roll, pitch, velocity_x, velocity_y, velocity_z])

        print(f"【 傾 き 】 Roll(左右): {roll:6.1f}° , Pitch(前後): {pitch:6.1f}° (水平=0°)")
        print(f"【 方 位 】 Heading: {heading:6.1f}° ({direction_str}方向)")
    else:
        print("【 傾 き 】 取得中...")
        print("【 方 位 】 取得中...")

    if lin_accel and lin_accel[0] is not None:
        threshold = 0.1 
        ax = lin_accel[0] if abs(lin_accel[0]) > threshold else 0.0
        ay = lin_accel[1] if abs(lin_accel[1]) > threshold else 0.0
        az = lin_accel[2] if abs(lin_accel[2]) > threshold else 0.0

        velocity_x += ax * dt
        velocity_y += ay * dt
        velocity_z += az * dt

        print(f"【加速度】 X: {ax:6.2f} m/s², Y: {ay:6.2f} m/s², Z: {az:6.2f} m/s²")
        print(f"【 速 度 】 X: {velocity_x:6.2f} m/s , Y: {velocity_y:6.2f} m/s , Z: {velocity_z:6.2f} m/s")
    else:
        print("【加速度】 取得中...")
        print("【 速 度 】 取得中...")

    if gyro and gyro[0] is not None:
        print(f"Gyroscope   - X: {gyro[0]:6.2f} /s, Y: {gyro[1]:6.2f} /s, Z: {gyro[2]:6.2f} /s")
    
    print(f"Calibration - Sys: {calib[0]}, Gyro: {calib[1]}, Accel: {calib[2]}, Mag: {calib[3]}")

# ==============================================================================
# [3] DFRobot GNSS センサー関連の関数
# ==============================================================================
def measure_gnss(csv_writer):
    """GNSSデータの取得・表示・CSV書き込み"""
    local_timestamp = time.time()  # 一番左の共通タイムスタンプ

    # 各データの取得
    utc = gnss.get_utc()
    date = gnss.get_date()
    lat = gnss.get_lat()
    lon = gnss.get_lon()
    alt = gnss.get_alt()
    sog = gnss.get_sog()
    cog = gnss.get_cog()
    sat = gnss.get_num_sta_used()

    # 一番左に local_timestamp を追加してCSV書き込み
    csv_writer.writerow([
        local_timestamp, 
        date.year, date.month, date.date, 
        utc.hour, utc.minute, utc.second, 
        lat.latitude_degree, lon.lonitude_degree, 
        alt, sog, cog, sat
    ])

    # 画面表示
    print("-------- GNSS --------")
    print("Satellites:", sat)
    print(f"Date: {date.year}/{date.month}/{date.date}")
    print(f"UTC: {utc.hour}:{utc.minute}:{utc.second}")
    print("Latitude:", lat.latitude_degree)
    print("Longitude:", lon.lonitude_degree)
    print("Altitude:", alt)
    print("Speed:", sog)
    print("Course:", cog)
    print("-" * 40)

# ==============================================================================
# メイン実行処理
# ==============================================================================
def main():
    # 各センサーの初期化処理
    init_sensors()
    
    print("Measurement started... Press Ctrl+C to stop.")
    print("-" * 60)

    # 各CSVファイルを開く
    f_dist = open("distance_log.csv", "w", newline="")
    f_bno = open("bno055_data.csv", "w", newline="")
    f_gnss = open("gnss_data.csv", "w", newline="")
    
    writer_dist = csv.writer(f_dist)
    writer_bno = csv.writer(f_bno)
    writer_gnss = csv.writer(f_gnss)
    
    # ヘッダー行の設定（すべての一番左に local_timestamp を追加）
    writer_dist.writerow(["local_timestamp", "S1_Timestamp", "S1_Distance_cm", "S2_Timestamp", "S2_Distance_cm"])
    writer_bno.writerow(["local_timestamp", "timestamp", "heading", "roll", "pitch", "velocity_x", "velocity_y", "velocity_z"])
    writer_gnss.writerow(["local_timestamp", "year", "month", "day", "hour", "minute", "second", "Latitude", "Longitude", "Altitude", "Speed", "Course", "Satellites"])

    try:
        time.sleep(0.5)

        while True:
            # 各処理を関数の羅列で実行
            measure_bno055(writer_bno)
            measure_gnss(writer_gnss)
            measure_ultrasonic(writer_dist)

            # データの即時書き込みを保証
            f_dist.flush()
            f_bno.flush()
            f_gnss.flush()

            # 元のGNSSコードの待機時間（2秒）に合わせるか、適宜調整してください
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nMeasurement stopped by User")

    finally:
        # 安全な終了処理
        GPIO.cleanup()
        f_dist.close()
        f_bno.close()
        f_gnss.close()

if __name__ == "__main__":
    main()