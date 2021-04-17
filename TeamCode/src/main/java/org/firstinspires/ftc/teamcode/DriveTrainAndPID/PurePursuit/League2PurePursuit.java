package org.firstinspires.ftc.teamcode.DriveTrainAndPID.PurePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class League2PurePursuit extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        RobotMovement.goToPosition(0, 100, 0.2, hardwareMap);
    }
}