package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WheelSubsystem extends SubsystemBase {
    private final Motor m_wheel;
    private boolean m_atTargetLocation = true;

    public WheelSubsystem(final HardwareMap hardwareMap, final String name, final boolean inverted) {
        m_wheel = new Motor(hardwareMap, name, Motor.GoBILDA.RPM_1150);
        m_wheel.setInverted(inverted);
        m_wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_wheel.resetEncoder();
        m_wheel.setRunMode(Motor.RunMode.PositionControl);
        m_wheel.setPositionCoefficient(0.05);
    }

    public void driveDistance(double power, double damp, double distance) {
        //m_wheel.setRunMode(Motor.RunMode.PositionControl);
        m_atTargetLocation = false;
        power = damp * power;
        m_wheel.resetEncoder();

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) ((m_wheel.getCurrentPosition())/2 + distance * 90);

        //Set target position
        m_wheel.setTargetPosition(encoderDistance);
        m_wheel.set(0);
        m_wheel.setPositionTolerance(13.6);
        m_wheel.set(power);


        while (!m_wheel.atTargetPosition()) {
            m_wheel.set(power);
        }

        //Stop and change modes back to normal
        m_wheel.set(0);
        m_wheel.resetEncoder();
        m_atTargetLocation = true;
    }

    public void resetEncoder() {
        m_wheel.resetEncoder();
    }

    public boolean isDone() {
        return m_atTargetLocation;
    }

    public void drivePower(double power, double damp) {
        m_wheel.setRunMode(Motor.RunMode.RawPower);
        power = damp * power;
        m_wheel.set(power);
    }
}
