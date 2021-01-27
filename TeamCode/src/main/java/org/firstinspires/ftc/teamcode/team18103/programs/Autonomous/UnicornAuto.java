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

        //AutoMode mode = AutoMode.One;
        AutoMode mode = robot.getDriveSubsystem().getVisionProcessing().getAutoMode();

        double currentPos = robot.getDriveSubsystem().getDataFusionY();

        if(mode == AutoMode.None) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 9) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
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
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 17) {
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
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    45 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() < currentPos + 8) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
        } else if(mode == AutoMode.One) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                if(Math.abs(robot.getDriveSubsystem().getDataFusionTheta()) > 5) {
                    if (0 - robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                        while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    } else {
                        while(0 - robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(-0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    }
                }
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while(robot.getDriveSubsystem().getDataFusionTheta() < 45) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            double currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) < currentDiagonalPosition + 6) {
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 7) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while(robot.getDriveSubsystem().getDataFusionTheta() > -3) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            double currentPosY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() > currentPosY - 20) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    45 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
            currentPosY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() < currentPosY + 17) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
        } else {
            double currentTheta = 0;
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.27);
                if(Math.abs(robot.getDriveSubsystem().getDataFusionTheta() - currentTheta) > 5) {
                    if (currentTheta - robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                        while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    } else {
                        while(currentTheta - robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(-0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    }
                }
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while (robot.getDriveSubsystem().getDataFusionTheta() > -30) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            double currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) < currentDiagonalPosition + 6) {
                robot.getDriveSubsystem().setDriveMotors(0.35);
                robot.loop(telemetry);
            }
            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 12) {
                robot.getDriveSubsystem().setDriveMotors(-0.35);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while(robot.getDriveSubsystem().getDataFusionTheta() < 30) {
                robot.getDriveSubsystem().setRotateMotors(0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 20) {
                robot.getDriveSubsystem().setDriveMotors(-0.35);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            while(robot.getDriveSubsystem().getDataFusionTheta() > 0) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            double currentPositionY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() > currentPositionY - 22) {
                robot.getDriveSubsystem().setDriveMotors(-0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(300);
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    45 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
            currentPositionY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() < currentPositionY + 17) {
                robot.getDriveSubsystem().setDriveMotors(0.30);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);

        }


    }
}