package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

//importing the statements for the code below
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp", group="Iterative TeamCode")
//@Disabled
public class TeleOp_Final extends OpMode {

    //defining all of the variables needed for the code
    private ElapsedTime runtime = new ElapsedTime();
    private Motor RBMotor, GlockMotor, CountGlockMotor, IntakeMotor, WheelSpinnyMotor;
    //Left is for LF Motor, Right is for RF Motor and Back is for LB Motor
    private MotorEx RFMotor, LFMotor, LBMotor;
    private RevIMU imu;
    private double lastAngles = 0;
    private boolean fieldRelativeMode = false;
    private double globalAngle, speed = 0.75;
    // define our trackwidth
    static final double TRACKWIDTH = 14.5;

    // convert ticks to inches
    static final double TICKS_TO_INCHES = 76.6;

    static final double centerWheelOffset = 0.5;

    private HolonomicOdometry diffOdom;


    @Override
    public void init() throws IllegalArgumentException {

        //grabbing the hardware from the expansion hubs, and the configuration
        LFMotor  = new MotorEx(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        LBMotor  = new MotorEx(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);
        RFMotor  = new MotorEx(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        RBMotor  = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);

        LFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        /*GlockMotor = new Motor(hardwareMap, "Glock Motor", Motor.GoBILDA.RPM_1150);
        CountGlockMotor = new Motor(hardwareMap, "Counter Glock Motor", Motor.GoBILDA.RPM_1150);
        IntakeMotor = new Motor(hardwareMap, "Intake Motor", Motor.GoBILDA.RPM_435);*/

        imu = new RevIMU(hardwareMap, "imu");

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        LFMotor.setInverted(false);
        LBMotor.setInverted(false);
        RFMotor.setInverted(true);
        RBMotor.setInverted(true);

        //setting up the IMU on the expansion hubs, for our use
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.init(parameters);
        // create our odometry
        diffOdom = new HolonomicOdometry(TRACKWIDTH, centerWheelOffset);

        diffOdom.updatePose(new Pose2d(0, 0, new Rotation2d(0)));
        LFMotor.encoder.reset();
        RFMotor.encoder.reset();
        LBMotor.encoder.reset();
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        /*if(gamepad1.right_bumper){
            GlockMotor.set(-1);
            CountGlockMotor.set(1);
        } else if(gamepad1.left_bumper){
            GlockMotor.set(1);
            CountGlockMotor.set(-1);
        } else{
            GlockMotor.set(0);
            CountGlockMotor.set(0);
        }

        if(gamepad1.right_trigger > 0){
            IntakeMotor.set(1);
        } else if(gamepad1.left_trigger > 0){
            IntakeMotor.set(-1);
        } else{
            IntakeMotor.set(0);
        }

        if(gamepad1.x){
            WheelSpinnyMotor.set(1);
        } else if(gamepad1.y){
            WheelSpinnyMotor.set(-1);
        } else{
            WheelSpinnyMotor.set(0);
        }*/

        diffOdom.update(LFMotor.encoder.getPosition(), RFMotor.encoder.getPosition(), LBMotor.encoder.getPosition());

        telemetry.addData("x position: ", diffOdom.getPose().getTranslation().getX() / TICKS_TO_INCHES);
        telemetry.addData("y position: ", diffOdom.getPose().getTranslation().getY() / TICKS_TO_INCHES);
        telemetry.addData("rotation: ", diffOdom.getPose().getRotation().getDegrees() / TICKS_TO_INCHES);

        if (gamepad1.b) {
            LFMotor.encoder.reset();
            RFMotor.encoder.reset();
            LBMotor.encoder.reset();
            diffOdom.updatePose(new Pose2d(0, 0, new Rotation2d(0)));
            do {
                LFMotor.set(0.5);
                LBMotor.set(0.5);
                RFMotor.set(0.5);
                RBMotor.set(0.5);
                diffOdom.update(LFMotor.encoder.getPosition(), RFMotor.encoder.getPosition(), LBMotor.encoder.getPosition());
            } while ((diffOdom.getPose().getTranslation().getX() / TICKS_TO_INCHES) < 30);
            LFMotor.set(0);
            LBMotor.set(0);
            RFMotor.set(0);
            RBMotor.set(0);
        }
        //defining the value to get from phones
        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;
        float slidesValue;

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
            double angle = getAngle();
            double tempX = (xValue * Math.cos(Math.toRadians(angle))) - (yValue * Math.sin(Math.toRadians(angle)));
            yValue = (xValue * Math.sin(Math.toRadians(angle))) + (yValue * Math.cos(Math.toRadians(angle)));
            xValue = tempX;
        }

        //getting the values for the powers for each motor
        LFPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        //applying the ramping up and ramping down features
        if (LFPower < 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
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

        slidesValue = gamepad2.left_stick_y;

        if (gamepad1.a){
            speed = 0.3;
        } else{
            speed = 0.75;
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