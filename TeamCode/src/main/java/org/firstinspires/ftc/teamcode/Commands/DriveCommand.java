package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem m_drive;
    private final Pose2d m_target;
    private final double m_power;

    public DriveCommand(DriveSubsystem drive, double distance, double angle, double power) {
        m_drive = drive;
        m_target = new Pose2d(new Translation2d(distance * Math.sin(angle), distance * Math.cos(angle)), new Rotation2d(Math.toRadians(angle)));
        m_power = power;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return m_drive.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        int[] direction = new int[4];
        direction[0] = 1;
        direction[1] = 1;
        direction[2] = 1;
        direction[3] = 1;
        m_drive.drive(0, direction);
    }
}
