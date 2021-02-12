package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class Blue extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 13.7;
    static final double WHEEL_DIAMETER = 4.0; // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "centerEncoder");

        // calculate multiplier
        TICKS_TO_INCHES = leftEncoder.encoder.getRevolutions() * WHEEL_DIAMETER * Math.PI;

        //pure pursuit command include start, general and end waypoint
        m_robotOdometry = new HolonomicOdometry(
                leftEncoder::getDistance, rightEncoder::getDistance, centerEncoder::getDistance, TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        m_odometry = new OdometrySubsystem(m_robotOdometry);

        //CODE FROM HERE
        Rotation2d zero = new Rotation2d(0);

        Pose2d startBlueOut = new Pose2d(102.0, 198.0, zero);
        Pose2d startBlueIn = new Pose2d(102.0, 175.0, zero);
        Waypoint start = new StartWaypoint(startBlueOut);

        Pose2d disks = new Pose2d(119, 187, zero);
        Waypoint disk = new EndWaypoint(disks, 30, 30, 4, 1, 1);
        Waypoint diskStart = new StartWaypoint(disks);
        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, start, disk);

        //Run ppCommand

        Pose2d square;
        Waypoint squar;
        int detection = 0;

        if (detection == 0) {
            square = new Pose2d(150, 210, zero); // Square A
        } else if (detection == 1) {
            //Pick up 1 disk
            square = new Pose2d(175, 187, zero); // Square B
        } else {
            //Pick up 3 disks
            square = new Pose2d(200, 210, zero); // Square C
        }

        squar = new EndWaypoint(square, 30, 30, 4, 1, 1);
        Waypoint squarStart = new StartWaypoint(square);

        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, diskStart, squar);

        //Run ppCommand
        //Place the wobble goal

        Pose2d powerTarget = new Pose2d(160.0, 187.0, zero); //Location of power target shooting and parking
        Waypoint powerTar = new EndWaypoint(powerTarget, 30, 30, 4, 1, 1);
        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, squarStart, powerTar);

        //Shoot power targets
    }
}

//PURE PURSUIT INFORMATION
//Origin: bottom-left
//Grid-size: 350*350 (might be different on the real field)