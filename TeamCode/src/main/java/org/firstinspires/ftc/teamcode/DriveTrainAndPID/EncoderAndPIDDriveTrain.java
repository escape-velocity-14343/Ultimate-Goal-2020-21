package org.firstinspires.ftc.teamcode.DriveTrainAndPID;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class EncoderAndPIDDriveTrain {
    //initializing the variables that the methods will need later on
    private Motor LFMotor, LBMotor, RFMotor, RBMotor;
    private PIDController pidRotate;
    private RevIMU imu;
    private double lastAngles = 0;
    private double globalAngle;
    private double damp = 0.5;
    private double distance_shorten = 2.6;

    public EncoderAndPIDDriveTrain(Motor m_LFMotor, Motor m_LBMotor, Motor m_RFMotor, Motor m_RBMotor, RevIMU m_imu){
        //defining the motors as the motor values that we get from the class
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

        LFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //creating the PID controller for use in the turning
        pidRotate = new PIDController(.003, .00003, 0);

        //setting the motor directions
        LFMotor.setInverted(false);
        LBMotor.setInverted(false);
        RFMotor.setInverted(true);
        RBMotor.setInverted(true);

        //defining and creating the IMU sensor for the robot to use
        this.imu = m_imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.init(parameters);
    }

    //resetting the angle in the IMU
    public void resetAngle() {
        imu.reset();
        lastAngles = 0;
        globalAngle = 0;
    }

    //getting the current angle of the IMU
    public double getAngle() {

        double angles = imu.getAngles()[0];

        double deltaAngle = angles - lastAngles;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //setting the motors, allowing the robot to drive forwards
    public void DriveForward(double power) {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        LFMotor.set(power);
        LBMotor.set(power);
        RFMotor.set(power);
        RBMotor.set(power);
    }

    //stop driving the robot
    public void StopDriving() {

        DriveForward(0);
    }

    //Drive forward using encoders
    public void DriveForwardDistance(double power, double distance)  {
        LBMotor.setRunMode(Motor.RunMode.PositionControl);
        LFMotor.setRunMode(Motor.RunMode.PositionControl);
        RFMotor.setRunMode(Motor.RunMode.PositionControl);
        RBMotor.setRunMode(Motor.RunMode.PositionControl);

        LBMotor.setPositionCoefficient(0.05);
        LFMotor.setPositionCoefficient(0.05);
        RFMotor.setPositionCoefficient(0.05);
        RBMotor.setPositionCoefficient(0.05);

        distance /= distance_shorten;

        power = damp * power;

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) ((LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        StopDriving();

        LFMotor.setPositionTolerance(13.6);
        LBMotor.setPositionTolerance(13.6);
        RFMotor.setPositionTolerance(13.6);
        RBMotor.setPositionTolerance(13.6);

        DriveForward(power);


        while (!LFMotor.atTargetPosition() && !LBMotor.atTargetPosition() && !RFMotor.atTargetPosition() && !RBMotor.atTargetPosition()) {
            DriveForward(power);
        }

        //Stop and change modes back to normal
        StopDriving();

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();
    }

    //turns left by setting the motors to do that
    public void TurnLeft(double power) {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        LFMotor.set(-power);
        LBMotor.set(-power);
        RFMotor.set(power);
        RBMotor.set(power);
    }

    //turns left using PID
    public void TurnLeftDegrees(double power, double degrees) throws InterruptedException   {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        do {
            power = pidRotate.performPID(getAngle());
            TurnLeft(power);
        } while (!pidRotate.onTarget());

        // turn the motors off.
        StopDriving();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //driving the robot backwards, by using the drove forwards method
    public void DriveBackward(double power) {
        power = damp * power;

        DriveForward(-power);
    }

    //driving the robot backwards using encoder ticks
    public void DriveBackwardDistance(double power, double distance)  {
        LBMotor.setRunMode(Motor.RunMode.PositionControl);
        LFMotor.setRunMode(Motor.RunMode.PositionControl);
        RFMotor.setRunMode(Motor.RunMode.PositionControl);
        RBMotor.setRunMode(Motor.RunMode.PositionControl);

        LBMotor.setPositionCoefficient(0.05);
        LFMotor.setPositionCoefficient(0.05);
        RFMotor.setPositionCoefficient(0.05);
        RBMotor.setPositionCoefficient(0.05);

        distance /= distance_shorten;

        power = damp * power;

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(-encoderDistance);
        LBMotor.setTargetPosition(-encoderDistance);
        RFMotor.setTargetPosition(-encoderDistance);
        RBMotor.setTargetPosition(-encoderDistance);

        StopDriving();

        LFMotor.setPositionTolerance(13.6);
        LBMotor.setPositionTolerance(13.6);
        RFMotor.setPositionTolerance(13.6);
        RBMotor.setPositionTolerance(13.6);

        DriveBackward(power);


        while (!LFMotor.atTargetPosition() && !LBMotor.atTargetPosition() && !RFMotor.atTargetPosition() && !RBMotor.atTargetPosition()) {
            DriveBackward(power);
        }

        //Stop and change modes back to normal
        StopDriving();

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();
    }

    //turning right, by setting the motors to the correct power
    public void TurnRight(double power) {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        LFMotor.set(power);
        LBMotor.set(power);
        RFMotor.set(-power);
        RBMotor.set(-power);
    }

    //turn right using the degree and PID
    public void TurnRightDegrees(double power, double degrees) throws InterruptedException {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        resetAngle();

        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        while (getAngle() == 0) {
            TurnRight(power);
            sleep(100);
        }

        do {
            power = pidRotate.performPID(getAngle());
            TurnLeft(power);
        } while (!pidRotate.onTarget());

        // turn the motors off.
        StopDriving();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //strafe right by setting the best powers
    public void StrafeRight(double power) {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        LFMotor.set(power);
        LBMotor.set(-power);
        RFMotor.set(-power);
        RBMotor.set(power);
    }

    //strafing right using the correct encoder distances
    public void StrafeRightDistance(double power, double distance) {
        LBMotor.setRunMode(Motor.RunMode.PositionControl);
        LFMotor.setRunMode(Motor.RunMode.PositionControl);
        RFMotor.setRunMode(Motor.RunMode.PositionControl);
        RBMotor.setRunMode(Motor.RunMode.PositionControl);

        LBMotor.setPositionCoefficient(0.05);
        LFMotor.setPositionCoefficient(0.05);
        RFMotor.setPositionCoefficient(0.05);
        RBMotor.setPositionCoefficient(0.05);

        distance /= distance_shorten;
        power = damp * power;

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(-encoderDistance);
        RFMotor.setTargetPosition(-encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        StopDriving();

        LFMotor.setPositionTolerance(13.6);
        LBMotor.setPositionTolerance(13.6);
        RFMotor.setPositionTolerance(13.6);
        RBMotor.setPositionTolerance(13.6);

        StrafeRight(power);


        while (!LFMotor.atTargetPosition() && !LBMotor.atTargetPosition() && !RFMotor.atTargetPosition() && !RBMotor.atTargetPosition()) {
            StrafeRight(power);
        }

        //Stop and change modes back to normal
        StopDriving();

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();
    }

    //strafing left
    public void StrafeLeft(double power) {
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        power = damp * power;

        LFMotor.set(-power);
        LBMotor.set(power);
        RFMotor.set(power);
        RBMotor.set(-power);
    }

    //strafing left a distance based on encoder values
    public void StrafeLeftDistance(double power, double distance) {
        LBMotor.setRunMode(Motor.RunMode.PositionControl);
        LFMotor.setRunMode(Motor.RunMode.PositionControl);
        RFMotor.setRunMode(Motor.RunMode.PositionControl);
        RBMotor.setRunMode(Motor.RunMode.PositionControl);

        LBMotor.setPositionCoefficient(0.05);
        LFMotor.setPositionCoefficient(0.05);
        RFMotor.setPositionCoefficient(0.05);
        RBMotor.setPositionCoefficient(0.05);

        distance /= distance_shorten;
        power = damp * power;

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(-encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(-encoderDistance);

        StopDriving();

        LFMotor.setPositionTolerance(13.6);
        LBMotor.setPositionTolerance(13.6);
        RFMotor.setPositionTolerance(13.6);
        RBMotor.setPositionTolerance(13.6);

        StrafeLeft(power);


        while (!LFMotor.atTargetPosition() && !LBMotor.atTargetPosition() && !RFMotor.atTargetPosition() && !RBMotor.atTargetPosition()) {
            StrafeLeft(power);
        }

        //Stop and change modes back to normal
        StopDriving();

        LFMotor.resetEncoder();
        LBMotor.resetEncoder();
        RFMotor.resetEncoder();
        RBMotor.resetEncoder();
    }
}
