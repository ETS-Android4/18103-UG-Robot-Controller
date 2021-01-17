package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;

@Autonomous
public class UnicornAuto extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.start();
        while (robot.getDriveSubsystem().getDataFusionY() < 18) {
            robot.getDriveSubsystem().setDriveMotors(0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);

        while (robot.getDriveSubsystem().getDataFusionTheta() < 15) {
            robot.getDriveSubsystem().setRotateMotors(0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);

        AutoMode mode = AutoMode.NoRings;//robot.getDriveSubsystem().getVisionProcessing().getAutoMode();

        while (robot.getDriveSubsystem().getDataFusionTheta() > 0) {
            robot.getDriveSubsystem().setRotateMotors(-0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);

        if(mode == AutoMode.NoRings) {
            while (robot.getDriveSubsystem().getDataFusionY() < mode.getDist()) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
        } else if(mode == AutoMode.FourRings) {
            while (robot.getDriveSubsystem().getDataFusionY() < mode.getDist()) {
                robot.getDriveSubsystem().setDriveMotors(0.5);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
        } else {
            while (robot.getDriveSubsystem().getDataFusionY() < mode.getDist()) {
                robot.getDriveSubsystem().setDriveMotors(0.5);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
        }

        while (robot.getDriveSubsystem().getDataFusionY() > 48) {
            robot.getDriveSubsystem().setDriveMotors(-0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);
        //while (robot.getDriveSubsystem().getDataFusionTheta() < 15) {
        //    robot.getDriveSubsystem().setRotateMotors(0.25);
        //    robot.loop(telemetry);
        //}
        /*robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);
        currentPos = robot.getDriveSubsystem().getDataFusionY();
        while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 25) {
            robot.getDriveSubsystem().setDriveMotors(-0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);
        while (robot.getDriveSubsystem().getDataFusionTheta() > 0) {
            robot.getDriveSubsystem().setRotateMotors(-0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);
        currentPos = robot.getDriveSubsystem().getDataFusionY();
        while (robot.getDriveSubsystem().getDataFusionY() < currentPos + 25) {
            robot.getDriveSubsystem().setDriveMotors(0.25);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);
        double currentPosX = robot.getDriveSubsystem().getDataFusionX();
        while(robot.getDriveSubsystem().getDataFusionX() > currentPosX - 2) {
            robot.getDriveSubsystem().setStrafeMotors(0.25);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(200);
        robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                45 - robot.getDriveSubsystem().getDataFusionY(),
                Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
        sleep(2000);
        robot.getIOSubsystem().runTransfer(true);
        sleep(2000);
        robot.getIOSubsystem().runTransfer(false);
        robot.getIOSubsystem().runOuttake(false);*/
        robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                120 - robot.getDriveSubsystem().getDataFusionY(),
                Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));

    }
}
