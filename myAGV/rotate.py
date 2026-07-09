import time
import sys

from pymycobot.myagv import MyAgv


PORT = "/dev/ttyAMA2"
BAUD = 115200
SPEED = 10
PAUSE = 1


def main():
    if len(sys.argv) != 3:
        return

    direction = sys.argv[1]
    seconds = float(sys.argv[2])

    agv = MyAgv(PORT, BAUD)

    agv.stop()
    time.sleep(PAUSE)

    if direction == "0":
        print(f"rotate cw {seconds}s")
        agv.clockwise_rotation(SPEED, seconds)
    elif direction == "1":
        print(f"rotate ccw {seconds}s")
        agv.counterclockwise_rotation(SPEED, seconds)
    else:
        print("invalid direction")
        return

    agv.stop()


if __name__ == "__main__":
    main()
