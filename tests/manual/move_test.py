import time

try:
    from pymycobot.mycobot import MyAgv
except ImportError:
    from pymycobot.myagv import MyAgv


PORT = "/dev/ttyAMA2"
BAUD = 115200
SPEED = 10


def main():
    agv = MyAgv(PORT, BAUD)

    agv.stop()
    time.sleep(1)

    print("forward 1s")
    agv.go_ahead(SPEED, 1)
    agv.stop()
    time.sleep(1)

    print("backward 1s")
    agv.retreat(SPEED, 1)
    agv.stop()


if __name__ == "__main__":
    main()
