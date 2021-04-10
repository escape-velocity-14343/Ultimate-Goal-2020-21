package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PIDController;


//Back up Auton that goes to the wall side of the bridge, and parks there
@Config
@Autonomous (name = "TESTING PID")
public class ShooterPIDTesting extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    MotorEx leftShooterMotor, rightShooterMotor;

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        leftShooterMotor = new MotorEx(hardwareMap, "Left Shooter Motor", Motor.GoBILDA.RPM_1150);
        rightShooterMotor = new MotorEx(hardwareMap, "Right Shooter Motor", Motor.GoBILDA.RPM_1150);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);

        leftShooterMotor.setRunMode(Motor.RunMode.VelocityControl);
        rightShooterMotor.setRunMode(Motor.RunMode.VelocityControl);

        leftShooterMotor.setVeloCoefficients(kP, kD, kI);
        waitForStart();

        //Running the code
        while (opModeIsActive()) {
            leftShooterMotor.set(0.8);
            rightShooterMotor.set(0.8);
            telemetry.addData("Left Shooter Motor Error", leftShooterMotor.getVelocity() - 0.8);
            telemetry.addData("Right Shooter Motor Error", rightShooterMotor.getVelocity() - 0.8);
            telemetry.addData("Left Shooter Motor", leftShooterMotor.getVelocity());
            telemetry.addData("Right Shooter Motor", rightShooterMotor.getVelocity());
            telemetry.log();
            telemetry.update();
        }
    }

}