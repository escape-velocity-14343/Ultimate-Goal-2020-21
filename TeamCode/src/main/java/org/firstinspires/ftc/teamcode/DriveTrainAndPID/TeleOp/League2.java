package org.firstinspires.ftc.teamcode.DriveTrainAndPID.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
//@Disabled
public class League2 extends CommandOpMode {
    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor bR, fL, fR, bL;
    private MotorEx leftEncoder, rightEncoder, perpEncoder;

    public static double TRACKWIDTH = 15.75;
    public static double CENTER_WHEEL_OFFSET = 0.5;
    public static double WHEEL_DIAMETER = 1.49606;

    public static double negLeft = 1;
    public static double negRight = -1;
    public static double negRear = 1;

    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 360*4;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void initialize() {
        bR = new Motor(hardwareMap, "RB Motor", Motor.GoBILDA.RPM_1150);
        fL = new Motor(hardwareMap, "LF Motor", Motor.GoBILDA.RPM_1150);
        fR = new Motor(hardwareMap, "RF Motor", Motor.GoBILDA.RPM_1150);
        bL = new Motor(hardwareMap, "LB Motor", Motor.GoBILDA.RPM_1150);


        leftEncoder = new MotorEx(hardwareMap, "Elevator Motor");
        rightEncoder = new MotorEx(hardwareMap, "Conveyor Motor");
        perpEncoder = new MotorEx(hardwareMap, "Right Shooter Motor");

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder.setDistancePerPulse(negLeft * DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(negRight * DISTANCE_PER_PULSE);
        perpEncoder.setDistancePerPulse(negRear * DISTANCE_PER_PULSE);

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        fL.setInverted(true);
        bL.setInverted(false);
        fR.setInverted(false);
        bR.setInverted(false);

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();

        //pure pursuit command include start, general and end waypoint
        m_robotOdometry = new HolonomicOdometry(
                leftEncoder::getDistance, rightEncoder::getDistance, perpEncoder::getDistance, TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        m_odometry = new OdometrySubsystem(m_robotOdometry);

        //Variables - rotation2D
        Rotation2d zero = new Rotation2d(0);

        //Variables - Pose2d
        Pose2d startBlue = new Pose2d(0, 0, zero);

        Pose2d square1 = new Pose2d(58, 27, zero);
        Pose2d square2 = new Pose2d(98, 35, zero);
        Pose2d square3 = new Pose2d(98, 35, zero);

        Pose2d checkRingsFar = new Pose2d(0, 12, zero);

        Pose2d powerShots1 = new Pose2d(33, -8, zero);
        Pose2d powerShots2 = new Pose2d(58, -8, zero);

        Pose2d wobble21 = new Pose2d(10, 32, zero);
        Pose2d wobble22 = new Pose2d(10, 31, zero); //22

        Pose2d ringsPick = new Pose2d(10, 12, zero);

        Pose2d highGoal = new Pose2d(58, 12, zero);

        //Waypoints
        Waypoint startBlueStart = new StartWaypoint(startBlue);

        Waypoint square1Start = new StartWaypoint(square1);
        Waypoint square1End = new EndWaypoint(square1, 1, 30, 4, 1, 1);

        Waypoint square2Start = new StartWaypoint(square2);
        Waypoint square2End = new EndWaypoint(square2, 1, 30, 4, 1, 1);

        Waypoint square3Start = new StartWaypoint(square3);
        Waypoint square3End = new EndWaypoint(square3, 1, 30, 4, 1, 1);

        Waypoint checkRingsFarStart = new StartWaypoint(checkRingsFar);
        Waypoint checkRingsFarEnd = new EndWaypoint(checkRingsFar, 1, 30, 4, 1, 1);

        Waypoint powerShots1Start = new StartWaypoint(powerShots1);
        Waypoint powerShots1End = new EndWaypoint(powerShots1, 1, 30, 4, 1, 1);

        Waypoint powerShots2Start = new StartWaypoint(powerShots2);
        Waypoint powerShots2End = new EndWaypoint(powerShots2, 1, 30, 4, 1, 1);

        Waypoint wobble21Start = new StartWaypoint(wobble21);
        Waypoint wobble21End = new EndWaypoint(wobble21, 1, 30, 4, 1, 1);

        Waypoint wobble22Start = new StartWaypoint(wobble22);
        Waypoint wobble22End = new EndWaypoint(wobble22, 1, 0, 4, 1, 1);

        Waypoint ringsPickStart = new StartWaypoint(ringsPick);
        Waypoint ringsPickEnd = new EndWaypoint(ringsPick, 1, 30, 4, 1, 1);

        //Waypoint highGoalStart = new StartWaypoint(highGoal);
        Waypoint highGoalEnd = new EndWaypoint(highGoal, 1, 30, 4, 1, 1);

        //Actual path

        waitForStart();
        if(!isStopRequested()) {
//            Path path1 = new Path(startBlueStart, checkRingsFarEnd);
//            path1.init();
//            path1.followPath(m_robotDrive, m_robotOdometry);

//            int num = 4;
//            Waypoint locationStart = square1Start;
//            Waypoint locationEnd = square1End;
//
//            if (num == 1) {
//                locationStart = square2Start;
//                locationEnd = square2End;
//            } else if (num == 4) {
//                locationStart = square3Start;
//                locationEnd = square3End;
//            }
//
//            Path path2 = new Path(checkRingsFarStart, powerShots1End);
//            path2.init();
//            path2.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path3 = new Path(powerShots1Start, powerShots2End);
//            path3.init();
//            path3.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path4 = new Path(powerShots2Start, locationEnd);
//            path4.init();
//            path4.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path5 = new Path(locationStart, wobble21End);
//            path5.init();
//            path5.followPath(m_robotDrive, m_robotOdometry);
//
            Path path6 = new Path(wobble21Start, wobble22End);
            path6.init();
            path6.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path7 = new Path(wobble22Start, ringsPickEnd);
//            path7.init();
//            path7.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path8 = new Path(ringsPickStart, locationEnd);
//            path8.init();
//            path8.followPath(m_robotDrive, m_robotOdometry);
//
//            Path path9 = new Path(locationStart, highGoalEnd);
//            path9.init();
//            path9.followPath(m_robotDrive, m_robotOdometry);
        }
    }
}

//PURE PURSUIT INFORMATION
//Add 48 to x value
//Subtract 25 from y value