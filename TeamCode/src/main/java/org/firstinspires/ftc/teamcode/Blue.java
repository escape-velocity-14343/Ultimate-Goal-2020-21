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
    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor bR;
    private MotorEx fL, fR, bL;

    // define our trackwidth
    static final double TRACKWIDTH = 14.5;
    // convert ticks to inches
    static final double TICKS_TO_INCHES = 76.6;
    static final double CENTER_WHEEL_OFFSET = 0.5;

    @Override
    public void initialize() {
        bR = new Motor(hardwareMap, "RB Motor");

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        fL = new MotorEx(hardwareMap, "LF Motor");
        fR = new MotorEx(hardwareMap, "RF Motor");
        bL = new MotorEx(hardwareMap, "LB Motor");

        //reversing the motors that need to be reversed, otherwise it sets it as forward
        fL.setInverted(false);
        bL.setInverted(false);
        fR.setInverted(true);
        bR.setInverted(true);

        fL.encoder.reset();
        fR.encoder.reset();
        bL.encoder.reset();

        //pure pursuit command include start, general and end waypoint
        m_robotOdometry = new HolonomicOdometry(
               fL::getDistance, fR::getDistance, bR::getDistance, TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        m_odometry = new OdometrySubsystem(m_robotOdometry);

        //CODE FROM HERE
        Rotation2d zero = new Rotation2d(0);

        Pose2d startT = new Pose2d(0, 0, zero);
        Waypoint start = new StartWaypoint(startT);

        Pose2d disks = new Pose2d(17, -11, zero);
        Waypoint disk = new EndWaypoint(disks, 30, 30, 4, 1, 1);
        Waypoint diskStart = new StartWaypoint(disks);
        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, start, disk);

        Pose2d highGoal = new Pose2d(58, -11, zero);
        Waypoint highGoalEnd = new EndWaypoint(highGoal, 30, 30, 4, 1, 1);
        Waypoint highGoalStart = new StartWaypoint(highGoal);

        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, diskStart, highGoalEnd);

        Pose2d powerShot = new Pose2d(58, -31, zero);
        Waypoint powerShotEnd = new EndWaypoint(powerShot, 30, 30, 4, 1, 1);
        Waypoint powerShotStart = new StartWaypoint(powerShot);

        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, highGoalStart, powerShotEnd);

        //Run ppCommand
        Pose2d square;
        Waypoint squar;
        int detection = 0;

        //Pose2d startBlueOut = new Pose2d(102.0, 198.0, zero);

        if (detection == 0) {
            square = new Pose2d(58, -1, zero); // Square A
        } else if (detection == 1) {
            //Pick up 1 disk
            square = new Pose2d(73, -17, zero); // Square B
        } else {
            //Pick up 3 disks
            square = new Pose2d(98, -12, zero); // Square C
        }

        squar = new EndWaypoint(square, 30, 30, 4, 1, 1);
        Waypoint squarStart = new StartWaypoint(square);

        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, powerShotStart, squar);

        //Run ppCommand
        //Place the wobble goal

        Pose2d powerTarget = new Pose2d(58, -11, zero); //Location of power target shooting and parking
        Waypoint powerTar = new EndWaypoint(powerTarget, 30, 30, 4, 1, 1);
        ppCommand = new PurePursuitCommand(m_robotDrive, m_odometry, squarStart, powerTar);

        //Shoot power targets
    }
}

//PURE PURSUIT INFORMATION
//Origin: bottom-left
//Grid-size: 350*350 (might be different on the real field)