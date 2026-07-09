import time

try:
    from pymycobot.mycobot import MyAgv
except ImportError:
    from pymycobot.myagv import MyAgv


PORT = "/dev/ttyAMA2"
BAUD = 115200
SPEED = 10
SIDE_TIME = 1
ROTATE_TIME = 5
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

    print("square")
    for side in range(4):
        run_step(agv, f"forward side {side + 1}", agv.go_ahead, SIDE_TIME)
        run_step(agv, f"turn cw {side + 1}", agv.clockwise_rotation, ROTATE_TIME)

    agv.stop()


if __name__ == "__main__":
    main()
