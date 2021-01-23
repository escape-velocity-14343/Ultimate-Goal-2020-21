package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private WheelSubsystem LFMotor, LRMotor, RFMotor, RRMotor;
    double damp = 0.5;

    public DriveSubsystem(final HardwareMap hardwareMap, final String leftFront, final String leftRear, final String rightFront, final String rightBack) {
        LFMotor = new WheelSubsystem(hardwareMap, leftFront, false);
        LRMotor = new WheelSubsystem(hardwareMap, leftRear, false);
        RFMotor = new WheelSubsystem(hardwareMap, rightFront, true);
        RRMotor = new WheelSubsystem(hardwareMap, rightBack, true);
    }

    /**
     * @param direction takes in an integer array with the direction that each wheel should be turning in the order,
     *                  the Left Front Motor, the Left Rear Motor, the Right Front Motor, and finally the Right Rear Motor
     */
    public void driveDistance(double power, double distance, int[] direction) {
        LFMotor.driveDistance(power, damp, distance);
        LRMotor.driveDistance(power, damp, distance);
        RFMotor.driveDistance(power, damp, distance);
        RRMotor.driveDistance(power, damp, distance);
    }

}
