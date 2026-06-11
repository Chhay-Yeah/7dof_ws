import sys
import time
import signal
import threading

from .lib.DM_motor import (
    DM_Motor_Type,
    Control_Mode,
    Control_Mode_Code,
    DmActData,
    Motor_Control,
)

running = threading.Event()
running.set()


def signal_handler(signum, frame):
    running.clear()
    sys.stderr.write(f"\nInterrupt signal ({signum}) received.\n")
    sys.stderr.flush()


signal.signal(signal.SIGINT, signal_handler)


def main():
    try:
        init_data1 = []
        canid7 = 0x07
        mstid7 = 0x17

        # Add motors to control – adjust motorType, mode, can_id, mst_id as needed
        init_data1.append(DmActData(
            motorType=DM_Motor_Type.DM4310_48V,
            mode=Control_Mode.POS_VEL_MODE,
            can_id=canid7,
            mst_id=mstid7))

        with Motor_Control(1000000, 1000000, "E067CA134F67746CCA5451F1BE23BAD8", init_data1) as control:
            control.switchControlMode(control.getMotor(canid7), Control_Mode_Code.POS_VEL)

            while running.is_set():
                desired_duration = 0.001  # seconds
                current_time = time.perf_counter()

                control.control_pos_vel(control.getMotor(canid7), 0.0, 2.0)

                pos = control.getMotor(canid7).Get_Position()
                vel = control.getMotor(canid7).Get_Velocity()
                tau = control.getMotor(canid7).Get_tau()
                interval = control.getMotor(canid7).getTimeInterval()
                print(
                    f"canid is: {canid7} pos: {pos} vel: {vel} effort: {tau} time(s): {interval}",
                    file=sys.stderr,
                )

                sleep_till = current_time + desired_duration
                now = time.perf_counter()
                if sleep_till > now:
                    time.sleep(sleep_till - now)

        print("The program exited safely.")

    except Exception as e:
        print(f"Error: hardware interface exception: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
