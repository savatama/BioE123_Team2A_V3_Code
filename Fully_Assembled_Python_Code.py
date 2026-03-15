import serial
import subprocess
import threading
import time

PORT = "/dev/cu.usbmodem2101"
BAUD = 9600
SONG_PATH = "/Users/savatamanaha/Desktop/rightround_v3centrifuge.mp3"

ser = None
running = True
song_process = None


def play_song():
    global song_process
    try:
        song_process = subprocess.Popen(["afplay", SONG_PATH])
    except Exception as e:
        print(f"\nCould not play song: {e}")


def test_stop():
    global song_process
    if song_process:
        song_process.terminate()
        song_process = None
        print("Song stopped.")


def read_serial():
    global running
    while running:
        try:
            line = ser.readline().decode(errors="ignore").strip()

            if line:
                print(f"\nArduino: {line}")

                if line == "RUN_DONE":
                    print("Run finished. Playing song.")
                    play_song()

                elif line == "SENSOR_FAULT":
                    print("Sensor fault detected.")

                print("> ", end="", flush=True)

        except Exception as e:
            print(f"\nSerial read error: {e}")
            running = False


def write_serial():
    global running
    while running:
        try:
            user_input = input("> ").strip()

            if not user_input:
                continue

            cmd = user_input.upper()

            if cmd == "TEST_SONG":
                print("Playing test song.")
                play_song()
                continue

            if cmd == "TEST_STOP":
                test_stop()
                continue

            ser.write((user_input + "\n").encode())

        except KeyboardInterrupt:
            running = False
            break
        except Exception as e:
            print(f"\nSerial write error: {e}")
            running = False
            break


def main():
    global ser

    print(f"Opening serial port: {PORT}")
    ser = serial.Serial(PORT, BAUD, timeout=1)

    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected.")
    print("Use this terminal like the Arduino Serial Monitor.")
    print()
    print("Example:")
    print(" 1500")
    print(" 30")
    print(" s")
    print()
    print("Commands:")
    print(" x -> stop / rampdown")
    print(" r -> reset")
    print(" KP 0.10")
    print(" KI 0.015")
    print(" TEST_SONG")
    print(" TEST_STOP")
    print()

    reader = threading.Thread(target=read_serial, daemon=True)
    reader.start()

    write_serial()

    try:
        ser.close()
    except:
        pass

    print("\nClosed serial connection.")


if __name__ == "__main__":
    main()
