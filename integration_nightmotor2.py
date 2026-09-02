import csv
import select
import sys
import termios
import time
import threading
import tty

import RPi.GPIO as GPIO
import adafruit_bno055
import board
import DFRobot_GNSS


# GPIO settings (BCM numbering)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Ultrasonic sensors: (name, TRIG, ECHO)
SENSORS = [
    ("Sensor 1", 23, 24),
    ("Sensor 2", 25, 26),
]

# Servo settings
SERVO1_PIN = 13
SERVO2_PIN = 4
PWM_FREQ = 50
STOP = 7.3
FORWARD = 10.0
REVERSE = 5.0
SERVO1_SLOW = 7.8
SERVO2_SLOW = 8.4

# LED setting: BCM GPIO27 (physical pin 13)
LED_PIN = 27

# BNO055 settings
BNO_ADDRESS = 0x28
BNO_REINIT_INTERVAL = 2.0
BNO_MEASUREMENT_INTERVAL = 0.2
ULTRASONIC_MEASUREMENT_INTERVAL = 1.0
GNSS_MEASUREMENT_INTERVAL = 1.0
GNSS_I2C_BUS = 1
GNSS_I2C_ADDRESS = 0x20

servo1 = None
servo2 = None
servo_command = "STOP"
servo1_duty = STOP
servo2_requested_duty = STOP
servo2_output_duty = STOP

led_state = False

bno_i2c = None
bno_sensor = None
bno_status = "Not initialized"
last_bno_reinit_attempt = 0.0
last_measurement_time = None
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

# DFRobot GNSS (GPS / BeiDou / GLONASS) settings
GPS_BeiDou_GLONASS = DFRobot_GNSS.GPS_BeiDou_GLONASS
gnss = DFRobot_GNSS.DFRobot_GNSS_I2C(
    bus=GNSS_I2C_BUS,
    addr=GNSS_I2C_ADDRESS,
)
gnss_status = "Not initialized"

old_terminal_settings = None
keyboard_enabled = False
keyboard_thread = None
keyboard_stop_event = threading.Event()
shutdown_requested = threading.Event()


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

    return (time.monotonic() - pulse_start) * 34300 / 2


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
        time.sleep(0.06)

    csv_writer.writerow([
        current_data["Sensor 1"]["time"],
        current_data["Sensor 1"]["distance"],
        current_data["Sensor 2"]["time"],
        current_data["Sensor 2"]["distance"],
    ])
    print("-" * 60)


def init_bno055(retries=5):
    global bno_i2c, bno_sensor, last_measurement_time, bno_status

    if bno_i2c is None:
        try:
            bno_i2c = board.I2C()
        except (OSError, RuntimeError, ValueError) as error:
            bno_status = f"I2C initialization error: {error}"
            print(bno_status)
            return False
        time.sleep(1.0)

    last_error = None
    for attempt in range(1, retries + 1):
        try:
            bno_sensor = adafruit_bno055.BNO055_I2C(
                bno_i2c, address=BNO_ADDRESS
            )
            time.sleep(0.7)
            last_measurement_time = time.monotonic()
            bno_status = "Ready"
            print("BNO055 initialization successful.")
            return True
        except (OSError, RuntimeError, ValueError) as error:
            last_error = error
            bno_sensor = None
            last_measurement_time = None
            bno_status = f"Initialization error: {error}"
            print(f"BNO055 initialization failed ({attempt}/{retries}): {error}")
            if attempt < retries:
                time.sleep(1.0)

    print(f"BNO055 could not be initialized: {last_error}")
    return False


def reinitialize_bno055_if_due():
    global last_bno_reinit_attempt, bno_status

    now = time.monotonic()
    if now - last_bno_reinit_attempt < BNO_REINIT_INTERVAL:
        return False
    last_bno_reinit_attempt = now
    bno_status = "Reinitializing"
    print("Reinitializing BNO055...")
    return init_bno055(retries=1)


def init_gnss():
    """Initialize the DFRobot GNSS module connected through I2C."""
    global gnss_status

    print("Initializing GNSS...")
    while not gnss.begin():
        gnss_status = "Initialization failed"
        print(
            "GNSS initialization failed. Check I2C wiring, module I2C mode, "
            "and address 0x20. Retrying..."
        )
        time.sleep(1)

    gnss.enable_power()
    gnss.set_gnss(GPS_BeiDou_GLONASS)
    gnss.rgb_on()
    gnss_status = "Ready"
    print("GNSS initialization successful.")


def measure_gnss(csv_writer):
    """Read GNSS values and append them to gnss_data.csv."""
    global gnss_status

    timestamp = time.time()
    try:
        utc = gnss.get_utc()
        date = gnss.get_date()
        lat = gnss.get_lat()
        lon = gnss.get_lon()
        alt = gnss.get_alt()
        sog = gnss.get_sog()
        cog = gnss.get_cog()
        sat = gnss.get_num_sta_used()

        csv_writer.writerow([
            timestamp,
            date.year, date.month, date.date,
            utc.hour, utc.minute, utc.second,
            lat.latitude_degree, lon.lonitude_degree,
            alt, sog, cog, sat,
        ])
        gnss_status = f"Ready ({sat} satellites)"

        print("-------- GNSS --------")
        print(f"Satellites: {sat}")
        print(f"Date (UTC): {date.year}/{date.month}/{date.date}")
        print(f"Time (UTC): {utc.hour:02d}:{utc.minute:02d}:{utc.second:02d}")
        print(f"Latitude: {lat.latitude_degree}")
        print(f"Longitude: {lon.lonitude_degree}")
        print(f"Altitude: {alt}")
        print(f"Speed: {sog}")
        print(f"Course: {cog}")
        print("-" * 60)
    except (OSError, RuntimeError, ValueError, AttributeError) as error:
        gnss_status = f"Read error: {error}"
        print(f"GNSS read error: {error}")


def get_compass_direction(heading):
    if heading is None:
        return "---"
    directions = [
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW",
    ]
    return directions[int((heading + 11.25) / 22.5) % 16]


def measure_bno055(csv_writer):
    global bno_sensor, velocity_x, velocity_y, velocity_z
    global last_measurement_time, bno_status

    timestamp = time.time()
    heading = roll = pitch = None
    ax = ay = az = None
    gyro = calibration = None
    communication_error = None

    if bno_sensor is None:
        reinitialize_bno055_if_due()

    if bno_sensor is not None:
        try:
            current_time = time.monotonic()
            dt = 0.0 if last_measurement_time is None else current_time - last_measurement_time
            last_measurement_time = current_time
            if dt < 0 or dt > 2.0:
                dt = 0.0

            euler = bno_sensor.euler
            linear_acceleration = bno_sensor.linear_acceleration
            gyro = bno_sensor.gyro
            calibration = bno_sensor.calibration_status

            if euler and all(value is not None for value in euler):
                heading, roll, pitch = euler

            if linear_acceleration and all(value is not None for value in linear_acceleration):
                threshold = 0.1
                ax, ay, az = [
                    value if abs(value) > threshold else 0.0
                    for value in linear_acceleration
                ]
                velocity_x += ax * dt
                velocity_y += ay * dt
                velocity_z += az * dt
            bno_status = "Ready"
        except (OSError, RuntimeError, ValueError) as error:
            communication_error = error
            bno_status = f"Communication error: {error}"
            bno_sensor = None
            last_measurement_time = None

    print("\033[2J\033[H", end="")
    print("--- Integrated sensor, servo, and LED control ---")
    print("w: forward | a: servo 1 slow | d: servo 2 slow | s: reverse")
    print("space: stop | e: LED ON/OFF | q: quit")
    print(f"Servo command: {servo_command}")
    print(f"Servo 1 duty: {servo1_duty:.2f}%")
    print(f"Servo 2 duty (requested/output): {servo2_requested_duty:.2f}% / {servo2_output_duty:.2f}%")
    print(f"LED (GPIO{LED_PIN}): {'ON' if led_state else 'OFF'}")
    print(f"GNSS: {gnss_status}")
    print("-" * 60)
    print(f"BNO055: {bno_status}")

    if communication_error is not None:
        print(f"I2C communication error: {communication_error}")
    if heading is not None:
        print(f"Heading: {heading:6.1f} deg ({get_compass_direction(heading)})")
        print(f"Roll: {roll:6.1f} deg, Pitch: {pitch:6.1f} deg")
    else:
        print("Orientation: measuring...")
    if ax is not None:
        print(f"Linear acceleration: X={ax:6.2f}, Y={ay:6.2f}, Z={az:6.2f} m/s^2")
        print(f"Velocity: X={velocity_x:6.2f}, Y={velocity_y:6.2f}, Z={velocity_z:6.2f} m/s")
    else:
        print("Acceleration/velocity: measuring...")
    if gyro and all(value is not None for value in gyro):
        print(f"Gyro: X={gyro[0]:6.2f}, Y={gyro[1]:6.2f}, Z={gyro[2]:6.2f} rad/s")
    if calibration and len(calibration) == 4:
        print(f"Calibration: Sys={calibration[0]}, Gyro={calibration[1]}, Accel={calibration[2]}, Mag={calibration[3]}")

    csv_writer.writerow([
        timestamp, heading, roll, pitch, ax, ay, az,
        velocity_x, velocity_y, velocity_z, servo_command,
        servo1_duty, servo2_requested_duty, servo2_output_duty, bno_status,
    ])


def init_servos():
    global servo1, servo2
    GPIO.setup(SERVO1_PIN, GPIO.OUT)
    GPIO.setup(SERVO2_PIN, GPIO.OUT)
    servo1 = GPIO.PWM(SERVO1_PIN, PWM_FREQ)
    servo2 = GPIO.PWM(SERVO2_PIN, PWM_FREQ)
    servo1.start(STOP)
    servo2.start(STOP)
    set_servo(STOP, STOP, "STOP")
    time.sleep(0.5)


def set_servo(s1, s2, command):
    global servo_command, servo1_duty, servo2_requested_duty, servo2_output_duty
    if servo1 is None or servo2 is None:
        return
    actual_servo2_duty = 2 * STOP - s2
    servo1.ChangeDutyCycle(s1)
    servo2.ChangeDutyCycle(actual_servo2_duty)
    servo_command = command
    servo1_duty = s1
    servo2_requested_duty = s2
    servo2_output_duty = actual_servo2_duty


def stop_servo_signal():
    global servo_command, servo1_duty, servo2_requested_duty, servo2_output_duty
    if servo1 is None or servo2 is None:
        return
    set_servo(STOP, STOP, "STOP")
    time.sleep(0.2)
    servo1.ChangeDutyCycle(0)
    servo2.ChangeDutyCycle(0)
    servo_command = "STOP (PWM OFF)"
    servo1_duty = servo2_requested_duty = servo2_output_duty = 0.0


def close_servos():
    global servo1, servo2
    if servo1 is not None and servo2 is not None:
        stop_servo_signal()
    if servo1 is not None:
        servo1.stop()
        servo1 = None
    if servo2 is not None:
        servo2.stop()
        servo2 = None


def init_led():
    global led_state
    GPIO.setup(LED_PIN, GPIO.OUT)
    led_state = False
    GPIO.output(LED_PIN, GPIO.LOW)


def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(LED_PIN, GPIO.HIGH if led_state else GPIO.LOW)
    print(f"LED: {'ON' if led_state else 'OFF'}")


def close_led():
    global led_state
    led_state = False
    GPIO.output(LED_PIN, GPIO.LOW)


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
    elif key == "e":
        toggle_led()
    elif key == "q":
        stop_servo_signal()
        return False
    return True


def init_keyboard():
    global old_terminal_settings, keyboard_enabled
    if not sys.stdin.isatty():
        print("Keyboard control is unavailable because standard input is not a terminal.")
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


def keyboard_listener():
    """Run in a separate thread so sensor reads never delay key handling."""
    while not keyboard_stop_event.is_set():
        readable, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not readable:
            continue

        key = sys.stdin.read(1).lower()
        if key == "\x03":
            shutdown_requested.set()
            keyboard_stop_event.set()
            return

        if not handle_servo_key(key):
            shutdown_requested.set()
            keyboard_stop_event.set()
            return


def start_keyboard_listener():
    """Start non-blocking keyboard control after terminal setup is complete."""
    global keyboard_thread
    if not keyboard_enabled:
        return

    keyboard_stop_event.clear()
    shutdown_requested.clear()
    keyboard_thread = threading.Thread(
        target=keyboard_listener,
        name="keyboard-listener",
        daemon=True,
    )
    keyboard_thread.start()


def stop_keyboard_listener():
    """Stop the keyboard thread before GPIO and terminal cleanup."""
    global keyboard_thread
    keyboard_stop_event.set()
    if keyboard_thread is not None and keyboard_thread.is_alive():
        keyboard_thread.join(timeout=0.2)
    keyboard_thread = None


def restore_keyboard():
    if keyboard_enabled and old_terminal_settings is not None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal_settings)


def close_bno055():
    global bno_i2c, bno_sensor, bno_status
    bno_sensor = None
    bno_status = "Closed"
    if bno_i2c is not None and hasattr(bno_i2c, "deinit"):
        try:
            bno_i2c.deinit()
        except Exception as error:
            print(f"I2C close error: {error}")
    bno_i2c = None


def main():
    f_dist = None
    f_bno = None
    f_gnss = None
    try:
        print("Integrated measurement started.")
        print("w/a/d/s: servo | space: stop | e: LED ON/OFF | q: quit")
        print("-" * 60)

        init_ultrasonic()
        init_bno055(retries=5)
        init_gnss()
        init_servos()
        init_led()
        init_keyboard()
        start_keyboard_listener()

        f_dist = open("distance_log.csv", "w", newline="", encoding="utf-8")
        f_bno = open("bno055_data.csv", "w", newline="", encoding="utf-8")
        f_gnss = open("gnss_data.csv", "w", newline="", encoding="utf-8")
        writer_dist = csv.writer(f_dist)
        writer_bno = csv.writer(f_bno)
        writer_gnss = csv.writer(f_gnss)
        writer_dist.writerow(["S1_Timestamp", "S1_Distance_cm", "S2_Timestamp", "S2_Distance_cm"])
        writer_bno.writerow([
            "timestamp", "heading", "roll", "pitch", "acceleration_x",
            "acceleration_y", "acceleration_z", "velocity_x", "velocity_y",
            "velocity_z", "servo_command", "servo1_duty",
            "servo2_requested_duty", "servo2_output_duty", "bno_status",
        ])
        writer_gnss.writerow([
            "local_timestamp", "year", "month", "day", "hour", "minute",
            "second", "latitude", "longitude", "altitude", "speed", "course",
            "satellites",
        ])

        next_bno_measurement = time.monotonic()
        next_ultrasonic_measurement = time.monotonic()
        next_gnss_measurement = time.monotonic()
        while not shutdown_requested.is_set():
            now = time.monotonic()
            if now >= next_bno_measurement:
                measure_bno055(writer_bno)
                f_bno.flush()
                next_bno_measurement = time.monotonic() + BNO_MEASUREMENT_INTERVAL
            if now >= next_ultrasonic_measurement:
                measure_ultrasonic(writer_dist)
                f_dist.flush()
                next_ultrasonic_measurement = time.monotonic() + ULTRASONIC_MEASUREMENT_INTERVAL
            if now >= next_gnss_measurement:
                measure_gnss(writer_gnss)
                f_gnss.flush()
                next_gnss_measurement = time.monotonic() + GNSS_MEASUREMENT_INTERVAL
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C.")
    except Exception as error:
        print(f"\nAn error occurred: {error}")
    finally:
        stop_keyboard_listener()
        try:
            close_servos()
        except Exception as error:
            print(f"Servo close error: {error}")
        try:
            close_led()
        except Exception as error:
            print(f"LED close error: {error}")
        restore_keyboard()
        if f_dist is not None:
            f_dist.close()
        if f_bno is not None:
            f_bno.close()
        if f_gnss is not None:
            f_gnss.close()
        close_bno055()
        GPIO.cleanup()
        print("GPIO and CSV files were closed.")


if __name__ == "__main__":
    main()
