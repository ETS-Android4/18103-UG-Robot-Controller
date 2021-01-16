package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;

@Autonomous
public class DriveNShootAuto extends LinearOpMode {
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.start();
        robot.getDriveSubsystem().setDriveMotors(0.5);
        do {
            //run
        } while (robot.getDriveSubsystem().getDataFusionY() < 72);
        robot.getDriveSubsystem().setDriveMotors(0);

        robot.getIOSubsystem().OuttakeFromPoint();
        sleep(1000);
        robot.getIOSubsystem().runTransfer(true);
        sleep(3000);
        robot.getIOSubsystem().runTransfer(false);
    }
}
