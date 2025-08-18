# !!! pip install -r requirements.txt !!!


"""
Enhanced Scanner System Main Program
==================================

This script coordinates the different phases of the scanning system:
1. Calibration
2. Initial Scan
3. Assurance Scan
4. Trajectory Following
"""

import pigpio
import queue
from src.hardware.lidar import LidarReader
from src.hardware.servo import ServoController
from src.hardware.stepper import StepperController
from src.phases.calibration import CalibrationPhase
from src.phases.initial_scan import InitialScanPhase
from src.phases.assurance_scan import AssuranceScanPhase
from src.phases.trajectory import TrajectoryPhase
from src.config.settings import LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, SERVO_PIN

def main():
    """Main program execution."""
    try:
        # Initialize hardware
        pi = pigpio.pi()
        if not pi.connected:
            print("Could not connect to pigpio daemon.")
            return

        # Set up components
        lidar_queue = queue.Queue()
        lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_queue)
        servo = ServoController(pi)
        stepper = StepperController()

        # Start LiDAR reading
        lidar_thread.start()

        # Execute calibration phase
        calibration = CalibrationPhase(stepper, servo, lidar_queue)
        average_distance = calibration.run()
        if average_distance is None:
            print("Calibration failed. Exiting...")
            return

        # Execute initial scan phase
        initial_scan = InitialScanPhase(stepper, servo, lidar_queue, average_distance)
        if not initial_scan.run():
            print("Initial scan failed. Exiting...")
            return

        # Execute assurance scan phase
        assurance = AssuranceScanPhase(stepper, servo, lidar_queue, average_distance)
        verified_points = assurance.run()
        if not verified_points:
            print("Assurance scan failed. Exiting...")
            return

        # Execute trajectory following phase
        trajectory = TrajectoryPhase(stepper, servo, lidar_queue, verified_points)
        final_trajectory = trajectory.run()

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {str(e)}")
    finally:
        print("Cleaning up...")
        if 'lidar_thread' in locals():
            lidar_thread.cleanup()
        if 'servo' in locals():
            servo.cleanup()
        if 'stepper' in locals():
            stepper.cleanup()
        if 'pi' in locals():
            pi.stop()

if __name__ == '__main__':
    main()
