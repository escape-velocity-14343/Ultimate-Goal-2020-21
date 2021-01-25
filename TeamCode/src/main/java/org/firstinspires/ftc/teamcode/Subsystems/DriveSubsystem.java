package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.RevIMU;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class DriveSubsystem extends SubsystemBase {
    private WheelSubsystem LFMotor, LRMotor, RFMotor, RRMotor;
    private RevIMU imu;
    double damp = 0.5;

    public DriveSubsystem() {
        LFMotor = new WheelSubsystem(hardwareMap, "LF Motor", false);
        LRMotor = new WheelSubsystem(hardwareMap, "LB Motor", false);
        RFMotor = new WheelSubsystem(hardwareMap, "RF Motor", true);
        RRMotor = new WheelSubsystem(hardwareMap, "RB Motor", true);
        imu = new RevIMU(hardwareMap, "imu");
        imu.init();
    }

    /**
     * @param direction takes in an integer array with the direction that each wheel should be turning in the order,
     *                  the Left Front Motor, the Left Rear Motor, the Right Front Motor, and finally the Right Rear Motor
     */
    public void drive(double power, int[] direction) {
        LFMotor.drivePower(direction[0] * power, damp);
        LRMotor.drivePower(direction[1] * power, damp);
        RFMotor.drivePower(direction[2] * power, damp);
        RRMotor.drivePower(direction[3] * power, damp);
    }

    /**
     * @param direction takes in an integer array with the direction that each wheel should be turning in the order,
     *                  the Left Front Motor, the Left Rear Motor, the Right Front Motor, and finally the Right Rear Motor
     */
    public void drive(double power, double distance, int[] direction) {
        LFMotor.driveDistance(direction[0] * power, damp, distance);
        LRMotor.driveDistance(direction[1] * power, damp, distance);
        RFMotor.driveDistance(direction[2] * power, damp, distance);
        RRMotor.driveDistance(direction[3] * power, damp, distance);
    }

    public void resetEncoders() {
        LFMotor.resetEncoder();
        LRMotor.resetEncoder();
        RFMotor.resetEncoder();
        RRMotor.resetEncoder();
    }

    public boolean isDone() {
        return LFMotor.isDone() && LRMotor.isDone() && RFMotor.isDone() && RRMotor.isDone();
    }

    public double getAngle() {
        return imu.getAngles()[0];
    }

    public void resetAngle() {
        imu.reset();
    }
}
