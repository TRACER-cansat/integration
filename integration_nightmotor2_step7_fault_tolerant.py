import csv
import select
import sys
import termios
import time
import threading
import queue
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
GNSS_INIT_TIMEOUT = 5.0
GNSS_INIT_RETRY_INTERVAL = 1.0
DISPLAY_INTERVAL = 0.5
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
last_status_display = 0.0

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
command_queue = queue.Queue()
servo_log_queue = queue.Queue()
servo_log_stop_event = threading.Event()
servo_log_thread = None


def advance_deadline(previous_deadline, interval, now):
    """Advance from the scheduled start time without accumulating task runtime drift."""
    next_deadline = previous_deadline + interval
    while next_deadline <= now:
        next_deadline += interval
    return next_deadline
ultrasonic_stop_event = threading.Event()
ultrasonic_thread = None
ultrasonic_writer = None
ultrasonic_file = None
active_ultrasonic_sensors = []
ultrasonic_sensor_status = {name: "Not initialized" for name, _trig, _echo in SENSORS}
gnss_stop_event = threading.Event()
gnss_thread = None
gnss_writer = None
gnss_file = None


def init_ultrasonic():
    """Initialize each ultrasonic sensor independently.

    A GPIO setup failure disables only the affected sensor. The returned value
    tells main whether at least one ultrasonic sensor can be measured.
    """
    global active_ultrasonic_sensors

    active_ultrasonic_sensors = []
    for name, trig, echo in SENSORS:
        try:
            GPIO.setup(trig, GPIO.OUT)
            GPIO.setup(echo, GPIO.IN)
            GPIO.output(trig, GPIO.LOW)
        except Exception as error:
            ultrasonic_sensor_status[name] = f"Unavailable: {error}"
            print(f"{name} initialization failed; continuing without it: {error}")
            continue
        ultrasonic_sensor_status[name] = "Ready"
        active_ultrasonic_sensors.append((name, trig, echo))

    if active_ultrasonic_sensors:
        time.sleep(0.1)
        return True

    print("All ultrasonic sensors are unavailable; continuing without them.")
    return False


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
        "Sensor 1": {"time": None, "distance": "unavailable"},
        "Sensor 2": {"time": None, "distance": "unavailable"},
    }

    for name, trig, echo in active_ultrasonic_sensors:
        measurement_time = time.time()
        current_data[name]["time"] = measurement_time
        try:
            dist = distance(trig, echo)
            if dist is None:
                current_data[name]["distance"] = "timeout"
                ultrasonic_sensor_status[name] = "Timeout"
                print(f"{name}: timeout")
            else:
                current_data[name]["distance"] = dist
                ultrasonic_sensor_status[name] = "Ready"
                print(f"{name}: {dist:.2f} cm")
        except Exception as error:
            current_data[name]["distance"] = "error"
            ultrasonic_sensor_status[name] = f"Read error: {error}"
            print(f"{name} read error; continuing: {error}")
        time.sleep(0.06)

    csv_writer.writerow([
        current_data["Sensor 1"]["time"],
        current_data["Sensor 1"]["distance"],
        current_data["Sensor 2"]["time"],
        current_data["Sensor 2"]["distance"],
    ])
    print("-" * 60)


def ultrasonic_worker():
    """Measure both ultrasonic sensors without blocking the control loop."""
    while not ultrasonic_stop_event.is_set() and not shutdown_requested.is_set():
        started = time.monotonic()
        try:
            measure_ultrasonic(ultrasonic_writer)
            if ultrasonic_file is not None:
                ultrasonic_file.flush()
        except Exception as error:
            print(f"Ultrasonic worker error: {error}")
        remaining = ULTRASONIC_MEASUREMENT_INTERVAL - (time.monotonic() - started)
        if remaining > 0:
            ultrasonic_stop_event.wait(remaining)


def start_ultrasonic_worker(writer, output_file):
    global ultrasonic_thread, ultrasonic_writer, ultrasonic_file
    ultrasonic_writer = writer
    ultrasonic_file = output_file
    ultrasonic_stop_event.clear()
    ultrasonic_thread = threading.Thread(
        target=ultrasonic_worker, name="ultrasonic-worker", daemon=True
    )
    ultrasonic_thread.start()


def stop_ultrasonic_worker():
    global ultrasonic_thread, ultrasonic_writer, ultrasonic_file
    ultrasonic_stop_event.set()
    if ultrasonic_thread is not None and ultrasonic_thread.is_alive():
        ultrasonic_thread.join(timeout=1.0)
    ultrasonic_thread = None
    ultrasonic_writer = None
    ultrasonic_file = None


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


def init_gnss(timeout=GNSS_INIT_TIMEOUT, retry_interval=GNSS_INIT_RETRY_INTERVAL):
    """Try GNSS for a bounded time, then let the rest of the system start."""
    global gnss_status

    print(f"Initializing GNSS (timeout {timeout:.1f}s)...")
    deadline = time.monotonic() + max(0.0, timeout)
    last_error = None

    while True:
        try:
            if gnss.begin():
                gnss.enable_power()
                gnss.set_gnss(GPS_BeiDou_GLONASS)
                gnss.rgb_on()
                gnss_status = "Ready"
                print("GNSS initialization successful.")
                return True
            last_error = "device did not acknowledge at I2C address 0x20"
        except Exception as error:
            last_error = str(error)

        now = time.monotonic()
        if now >= deadline:
            gnss_status = f"Unavailable: {last_error}"
            print(f"GNSS unavailable after {timeout:.1f}s; continuing without it: {last_error}")
            return False

        sleep_time = min(max(0.0, retry_interval), deadline - now)
        if sleep_time > 0:
            time.sleep(sleep_time)


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


def gnss_worker():
    while not gnss_stop_event.is_set() and not shutdown_requested.is_set():
        started = time.monotonic()
        try:
            measure_gnss(gnss_writer)
            if gnss_file is not None:
                gnss_file.flush()
        except Exception as error:
            print(f"GNSS worker error: {error}")
        remaining = GNSS_MEASUREMENT_INTERVAL - (time.monotonic() - started)
        if remaining > 0:
            gnss_stop_event.wait(remaining)


def start_gnss_worker(writer, output_file):
    global gnss_thread, gnss_writer, gnss_file
    gnss_writer, gnss_file = writer, output_file
    gnss_stop_event.clear()
    gnss_thread = threading.Thread(target=gnss_worker, name="gnss-worker", daemon=True)
    gnss_thread.start()


def stop_gnss_worker():
    global gnss_thread, gnss_writer, gnss_file
    gnss_stop_event.set()
    if gnss_thread is not None and gnss_thread.is_alive():
        gnss_thread.join(timeout=1.0)
    gnss_thread = gnss_writer = gnss_file = None


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
    global last_measurement_time, bno_status, last_status_display

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

    display_now = time.monotonic()
    if display_now - last_status_display >= DISPLAY_INTERVAL:
        last_status_display = display_now
        orientation = "measuring"
        if heading is not None:
            orientation = f"H={heading:.1f} R={roll:.1f} P={pitch:.1f}"
        acceleration = "measuring"
        if ax is not None:
            acceleration = f"A=({ax:.2f},{ay:.2f},{az:.2f}) V=({velocity_x:.2f},{velocity_y:.2f},{velocity_z:.2f})"
        print(
            f"servo={servo_command} led={'ON' if led_state else 'OFF'} "
            f"gnss={gnss_status} bno={bno_status} {orientation} {acceleration}"
        )
        if communication_error is not None:
            print(f"I2C communication error: {communication_error}")

    csv_writer.writerow([
        timestamp, heading, roll, pitch, ax, ay, az,
        velocity_x, velocity_y, velocity_z, bno_status,
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
    servo_log_queue.put((time.time(), command, s1, s2, actual_servo2_duty))


def servo_log_worker(csv_writer, output_file):
    while not servo_log_stop_event.is_set() or not servo_log_queue.empty():
        try:
            row = servo_log_queue.get(timeout=0.05)
        except queue.Empty:
            continue
        csv_writer.writerow(row)
        output_file.flush()
        servo_log_queue.task_done()


def start_servo_log_worker(csv_writer, output_file):
    global servo_log_thread
    servo_log_stop_event.clear()
    servo_log_thread = threading.Thread(
        target=servo_log_worker,
        args=(csv_writer, output_file),
        name="servo-log-worker",
        daemon=True,
    )
    servo_log_thread.start()


def stop_servo_log_worker():
    global servo_log_thread
    servo_log_stop_event.set()
    if servo_log_thread is not None and servo_log_thread.is_alive():
        servo_log_thread.join(timeout=1.0)
    servo_log_thread = None


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

        if key == "\x03":
            shutdown_requested.set()
            keyboard_stop_event.set()
            return
        command_queue.put(key)
        if key == "q":
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


def process_command_queue():
    while True:
        try:
            key = command_queue.get_nowait()
        except queue.Empty:
            return
        if not handle_servo_key(key):
            shutdown_requested.set()


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


def start_available_sensor_workers(
    ultrasonic_ready,
    gnss_ready,
    writer_dist,
    f_dist,
    writer_gnss,
    f_gnss,
):
    """Start only the sensor workers whose initialization succeeded."""
    if ultrasonic_ready:
        start_ultrasonic_worker(writer_dist, f_dist)
    else:
        print("Ultrasonic worker disabled because no sensor initialized.")

    if gnss_ready:
        start_gnss_worker(writer_gnss, f_gnss)
    else:
        print("GNSS worker disabled; other processing remains active.")


def main():
    f_dist = None
    f_bno = None
    f_gnss = None
    f_servo = None
    try:
        print("Integrated measurement started.")
        print("w/a/d/s: servo | space: stop | e: LED ON/OFF | q: quit")
        print("-" * 60)

        ultrasonic_ready = init_ultrasonic()
        init_bno055(retries=5)
        gnss_ready = init_gnss()
        init_servos()
        init_led()
        init_keyboard()
        start_keyboard_listener()

        f_dist = open("distance_log.csv", "w", newline="", encoding="utf-8")
        f_bno = open("bno055_data.csv", "w", newline="", encoding="utf-8")
        f_gnss = open("gnss_data.csv", "w", newline="", encoding="utf-8")
        f_servo = open("servo_commands.csv", "w", newline="", encoding="utf-8")
        writer_dist = csv.writer(f_dist)
        writer_bno = csv.writer(f_bno)
        writer_gnss = csv.writer(f_gnss)
        writer_servo = csv.writer(f_servo)
        writer_dist.writerow(["S1_Timestamp", "S1_Distance_cm", "S2_Timestamp", "S2_Distance_cm"])
        writer_bno.writerow([
            "timestamp", "heading", "roll", "pitch", "acceleration_x",
            "acceleration_y", "acceleration_z", "velocity_x", "velocity_y",
            "velocity_z", "bno_status",
        ])
        writer_gnss.writerow([
            "local_timestamp", "year", "month", "day", "hour", "minute",
            "second", "latitude", "longitude", "altitude", "speed", "course",
            "satellites",
        ])
        writer_servo.writerow([
            "timestamp", "command", "servo1_duty",
            "servo2_requested_duty", "servo2_output_duty",
        ])

        start_servo_log_worker(writer_servo, f_servo)
        start_available_sensor_workers(
            ultrasonic_ready,
            gnss_ready,
            writer_dist,
            f_dist,
            writer_gnss,
            f_gnss,
        )

        next_bno_measurement = time.monotonic()
        while not shutdown_requested.is_set():
            process_command_queue()
            now = time.monotonic()
            if now >= next_bno_measurement:
                measure_bno055(writer_bno)
                f_bno.flush()
                next_bno_measurement = advance_deadline(
                    next_bno_measurement, BNO_MEASUREMENT_INTERVAL, time.monotonic()
                )
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nStopped with Ctrl+C.")
    except Exception as error:
        print(f"\nAn error occurred: {error}")
    finally:
        stop_servo_log_worker()
        stop_gnss_worker()
        stop_ultrasonic_worker()
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
        if f_servo is not None:
            f_servo.close()
        close_bno055()
        GPIO.cleanup()
        print("GPIO and CSV files were closed.")


if __name__ == "__main__":
    main()
