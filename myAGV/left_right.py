import time

try:
    from pymycobot.mycobot import MyAgv
except ImportError:
    from pymycobot.myagv import MyAgv


PORT = "/dev/ttyAMA2"
BAUD = 115200
SPEED = 10
PAUSE = 1


def run_step(agv, label, action, duration):
    print(label)
    action(SPEED, duration)
    agv.stop()
    time.sleep(PAUSE)


def main():
    agv = MyAgv(PORT, BAUD)

    agv.stop()
    time.sleep(PAUSE)

    run_step(agv, "left 1s", agv.pan_left, 1)
    run_step(agv, "right 1s", agv.pan_right, 1)

    agv.stop()


if __name__ == "__main__":
    main()
