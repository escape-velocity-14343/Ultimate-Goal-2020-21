package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

//importing the statements for the code below
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




//importing the statements for the code below
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="TeleOp", group="Iterative TeamCode")
//@Disabled
public class TeleOp_Final extends OpMode {

    //defining all of the variables needed for the code
    private ElapsedTime runtime = new ElapsedTime();
    private Motor LFMotor, LBMotor, RFMotor, RBMotor, conveyorMotor, elevatorMotor, leftShooterMotor, rightShooterMotor;
    private Servo rightIntakeDownServo, leftIntakeDownServo, wobbleGoalClawServo;
    private CRServo liftWobbleGoalServo;
    private RevIMU imu;
    private double lastAngles = 0;
    private boolean fieldRelativeMode = false;
    private double globalAngle, speed = 0.75;
    private double shooterSpeed = 0.8;
    private boolean hasBeenPushedX = false, hasBeenPushedY = false;


    @Override
    public void init() throws IllegalArgumentException {

        //grabbing the hardware from the expansion hubs, and the configuration
        LFMotor  = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);
        conveyorMotor = new Motor(hardwareMap, "Conveyor Motor", Motor.GoBILDA.RPM_435);
        elevatorMotor = new Motor(hardwareMap, "Elevator Motor", Motor.GoBILDA.RPM_84);
        leftShooterMotor = new Motor(hardwareMap, "Left Shooter Motor", Motor.GoBILDA.RPM_1150);
        rightShooterMotor = new Motor(hardwareMap, "Right Shooter Motor", Motor.GoBILDA.RPM_1150);


        liftWobbleGoalServo = hardwareMap.crservo.get("Lift Wobble Goal Servo");
        wobbleGoalClawServo = hardwareMap.get(Servo.class, "Wobble Goal Claw Servo");
        rightIntakeDownServo = hardwareMap.get(Servo.class, "Right Intake Down Servo");
        leftIntakeDownServo = hardwareMap.get(Servo.class, "Left Intake Down Servo");

        imu = new RevIMU(hardwareMap, "imu");

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        LFMotor.setInverted(false);
        LBMotor.setInverted(false);
        RFMotor.setInverted(true);
        RBMotor.setInverted(true);
        conveyorMotor.setInverted(true);
        elevatorMotor.setInverted(true);
        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);

        rightIntakeDownServo.setDirection(Servo.Direction.REVERSE);
        leftIntakeDownServo.setDirection(Servo.Direction.FORWARD);
        wobbleGoalClawServo.setDirection(Servo.Direction.REVERSE);
        liftWobbleGoalServo.setDirection(CRServo.Direction.FORWARD);

        LFMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        conveyorMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setting up the IMU on the expansion hubs, for our use
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.init(parameters);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        //defining the value to get from phones
        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;

        //checking to see if field relative mode is on
        if (gamepad1.back) {
            resetAngle();
            fieldRelativeMode = !fieldRelativeMode;
        }

        telemetry.addData("FieldRelative?", fieldRelativeMode);

        //getting the movement values from the gamepad
        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        //changing the values for the field relative mode
        if (fieldRelativeMode){
            LFMotor.setInverted(true);
            LBMotor.setInverted(true);
            RFMotor.setInverted(false);
            RBMotor.setInverted(false);
            /*double angle = getAngle();
            double tempX = (xValue * Math.cos(Math.toRadians(angle))) - (yValue * Math.sin(Math.toRadians(angle)));
            yValue = (xValue * Math.sin(Math.toRadians(angle))) + (yValue * Math.cos(Math.toRadians(angle)));
            xValue = tempX;*/
        } else {
            LFMotor.setInverted(false);
            LBMotor.setInverted(false);
            RFMotor.setInverted(true);
            RBMotor.setInverted(true);
        }

        //getting the values for the powers for each motor
        LFPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        //applying the ramping up and ramping down features
        if (LFPower < 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        }

        if (LBPower < 0){
            LBPower = (float) -Math.pow(Math.abs(LBPower),2);
        } else if (LBPower > 0){
            LBPower = (float) Math.pow(Math.abs(LBPower),2);
        }

        if (RFPower < 0){
            RFPower = (float) -Math.pow(Math.abs(RFPower),2);
        } else if (RFPower > 0){
            RFPower = (float) Math.pow(Math.abs(RFPower),2);
        }

        if (RBPower < 0){
            RBPower = (float) -Math.pow(Math.abs(RBPower),2);
        } else if (RBPower > 0){
            RBPower = (float) Math.pow(Math.abs(RBPower),2);
        }

        //game pad 1 left trigger is intake and right trigger is shooting
        //trigger values are all in the float
        //need to set up motors, set their inversion factor, set speeds, based on trigger values
        //convereyr is counter, elevator is counter, shooting right is counter and shooting left is clock
        //


        if (gamepad1.a){
            speed = 0.3;
        } else{
            speed = 0.75;
        }

        //shooters
        if (gamepad1.right_trigger > 0.0){
            rightShooterMotor.set(shooterSpeed);
            leftShooterMotor.set(shooterSpeed);
        } else{
            rightShooterMotor.set(0.0);
            leftShooterMotor.set(0.0);
        }

        //conveyor & elevator
        if (gamepad1.left_trigger > 0.0){
            conveyorMotor.set(1.0);

        } else{
            conveyorMotor.set(0.0);
        }


        //revers converyor and elevator to unstuck smth.
        if (gamepad1.left_bumper){
            conveyorMotor.set(-1.0);
        }

        if (gamepad1.x){
            rightIntakeDownServo.setPosition(Servo.MAX_POSITION);
            leftIntakeDownServo.setPosition(Servo.MAX_POSITION);
        }

        if (gamepad2.x && !hasBeenPushedX){
            shooterSpeed = shooterSpeed - 0.005;
            hasBeenPushedX = true;
        } else {
            hasBeenPushedX = false;
        }

        if (gamepad2.y && !hasBeenPushedY){
            shooterSpeed = shooterSpeed + 0.005;
            hasBeenPushedY = true;
        } else {
            hasBeenPushedY = false;
        }

        telemetry.addData("Shooter Speed: ", shooterSpeed);

        if (gamepad1.b){
            wobbleGoalClawServo.setPosition(Servo.MAX_POSITION-Servo.MIN_POSITION/2+Servo.MIN_POSITION);
        }

        if (gamepad1.y){
            wobbleGoalClawServo.setPosition(Servo.MIN_POSITION);
        }

        if (gamepad1.right_bumper) {
            liftWobbleGoalServo.setPower(1.0);
        } else {
            liftWobbleGoalServo.setPower(0.0);
        }

        if (gamepad1.right_bumper || gamepad1.right_trigger > 0.0) {
            elevatorMotor.set(1);
        } else if (gamepad1.left_bumper){
            elevatorMotor.set(-1);
        } else {
            elevatorMotor.set(0.0);
        }


        //setting the powers for each of the motors
        LBMotor.setRunMode(Motor.RunMode.RawPower);
        LFMotor.setRunMode(Motor.RunMode.RawPower);
        RFMotor.setRunMode(Motor.RunMode.RawPower);
        RBMotor.setRunMode(Motor.RunMode.RawPower);

        LFMotor.set(Range.clip(LFPower, -speed, speed));
        LBMotor.set(Range.clip(LBPower, -speed, speed));
        RFMotor.set(Range.clip(RFPower, -speed, speed));
        RBMotor.set(Range.clip(RBPower, -speed, speed));

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    @Override
    public void stop() {
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
}