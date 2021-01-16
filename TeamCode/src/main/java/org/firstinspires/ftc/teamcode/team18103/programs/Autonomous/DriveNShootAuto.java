package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;

/*
 * Author: Akhil G
 */

@Disabled
@Autonomous
public class DriveNShootAuto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.start();
        robot.getDriveSubsystem().setDriveMotors(0.5);


    }
}