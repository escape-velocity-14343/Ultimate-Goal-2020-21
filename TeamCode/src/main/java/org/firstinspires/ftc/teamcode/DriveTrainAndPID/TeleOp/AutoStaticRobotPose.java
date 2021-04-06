package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.EncoderAndPIDDriveTrain;

@TeleOp
//@Disabled
public class AutoStaticRobotPose extends LinearOpMode {

    Motor LFMotor, LBMotor, RFMotor, RBMotor;
    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;
    EncoderAndPIDDriveTrain drive;
    RevIMU imu;

    public static final double TRACKWIDTH = 14.5;
    public static final double CENTER_WHEEL_OFFSET = 0.5;
    public static final double WHEEL_DIAMETER = 1.49606;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 360;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "Elevator Motor");
        rightEncoder = new MotorEx(hardwareMap, "Conveyor Motor");
        perpEncoder = new MotorEx(hardwareMap, "Right Shooter Motor");

        LFMotor  = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);

        imu = new RevIMU(hardwareMap, "imu");

        drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        waitForStart();
        PositionTracker.robotPose = odometry.getPose();
        telemetry.addData("X Position in inches: ", PositionTracker.robotPose.getX());
        telemetry.addData("Y Position in inches: ", PositionTracker.robotPose.getY());
        telemetry.addData("Rotational Position in degrees: ", PositionTracker.robotPose.getRotation().getDegrees());
        telemetry.update();

        drive.DriveForwardDistance(1, 35);

        while (!isStopRequested()) {
            // run autonomous

            // update positions
            odometry.updatePose();
            PositionTracker.robotPose = odometry.getPose();

            telemetry.addData("X Position in inches: ", PositionTracker.robotPose.getTranslation().getX());
            telemetry.addData("Y Position in inches: ", PositionTracker.robotPose.getTranslation().getY());
            telemetry.addData("Rotational Position in degrees: ", PositionTracker.robotPose.getRotation().getDegrees());
            telemetry.update();
        }
    }

}
