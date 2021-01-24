package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;

@Autonomous
public class UnicornAuto2 extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        // Init & Start
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.start();
        // Vision
        AutoMode mode = robot.getDriveSubsystem().getVisionProcessing().getAutoMode();
        // Drive to Square
        robot.getDriveSubsystem().CustomDriveStraight(mode.getDist());
        // Rotating the WG in the Square
        robot.getDriveSubsystem().CustomDriveRotate(mode.getAngle());
        robot.getDriveSubsystem().CustomDriveStraight(mode.getDist() + 6);
        robot.getDriveSubsystem().CustomDriveStraight(mode.getDist());
        robot.getDriveSubsystem().CustomDriveRotate(0);
        // Preparation to Shoot
        robot.getDriveSubsystem().CustomDriveStraight(75);
        robot.getDriveSubsystem().CustomDriveStrafe(12);
        // Shoot
        robot.getDriveSubsystem().rotateToShootingAngle();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < 1.5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    108 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
        }
        while (timer.seconds() < 5) {
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    108 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            robot.getIOSubsystem().runTransfer(true);
        }

        robot.getIOSubsystem().runTransfer(false);
        robot.getIOSubsystem().runOuttake(false);
        // Park
        robot.getDriveSubsystem().CustomDriveStraight(80);
    }
}