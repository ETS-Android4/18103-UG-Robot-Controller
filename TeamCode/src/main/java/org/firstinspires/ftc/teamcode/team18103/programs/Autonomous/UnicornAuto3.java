package org.firstinspires.ftc.teamcode.team18103.programs.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team18103.src.Robot;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;

@Autonomous
public class UnicornAuto3 extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        waitForStart();
        robot.start();

        AutoMode mode = robot.getDriveSubsystem().getVisionProcessing().getAutoMode();
        while (robot.getDriveSubsystem().getDataFusionY() < 18) {
            robot.getDriveSubsystem().setDriveMotors(0.5);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(500);

        /*while (robot.getDriveSubsystem().getDataFusionTheta() < 15) {
            robot.getDriveSubsystem().setRotateMotors(0.25);
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

         */

        double currentPos = robot.getDriveSubsystem().getDataFusionY();

        if(mode == AutoMode.None) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.5);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            currentPos = robot.getDriveSubsystem().getDataFusionY();
            while (robot.getDriveSubsystem().getDataFusionY() > currentPos - 9) {
                robot.getDriveSubsystem().setDriveMotors(-0.5);
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
                robot.getDriveSubsystem().setDriveMotors(-0.5);
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
                robot.getDriveSubsystem().setDriveMotors(0.5);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);

            //double targetAngle = robot.getDriveSubsystem().rotateToShootingAngle();
            //telemetry.addData("targetAngle", targetAngle);
            //double dist = Math.abs(targetAngle - robot.getDriveSubsystem().getDataFusionTheta());

            //while (Math.abs(targetAngle - robot.getDriveSubsystem().getDataFusionTheta()) > 15) {
            //    robot.getDriveSubsystem().POVMecanumDrive(0, 0,
            //            (targetAngle - robot.getDriveSubsystem().getDataFusionTheta())/dist, DriveMode.Balanced);
            //    robot.loop(telemetry);
            //}
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
        } else if(mode == AutoMode.One) {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.5);
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
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 6) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
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
            double currentPosY = robot.getDriveSubsystem().getDataFusionY();
            while(robot.getDriveSubsystem().getDataFusionY() > currentPosY - 20) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(500);
            //robot.getDriveSubsystem().rotateToShootingAngle();
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    45 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
        } else {
            while (robot.getDriveSubsystem().getDataFusionY() < currentPos + mode.getDist() + 4) {
                robot.getDriveSubsystem().setDriveMotors(0.5);
                if(Math.abs(robot.getDriveSubsystem().getDataFusionTheta()) > 5) {
                    if (robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                        while(robot.getDriveSubsystem().getDataFusionTheta() < 0) {
                            robot.getDriveSubsystem().setRotateMotors(0.25);
                            robot.loop(telemetry);
                        }
                        robot.getDriveSubsystem().setDriveMotors(0);
                        sleep(300);
                    } else {
                        while(robot.getDriveSubsystem().getDataFusionTheta() > 0) {
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
                robot.getDriveSubsystem().setDriveMotors(0.25);
                robot.loop(telemetry);
            }
            currentDiagonalPosition = Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2));
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 12) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
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
            while(Math.sqrt(Math.pow(robot.getDriveSubsystem().getDataFusionY(), 2) + Math.pow(robot.getDriveSubsystem().getDataFusionX(), 2)) > currentDiagonalPosition - 15) {
                robot.getDriveSubsystem().setDriveMotors(-0.25);
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
            while(robot.getDriveSubsystem().getDataFusionY() > currentPositionY - 25) {
                robot.getDriveSubsystem().setDriveMotors(-0.5);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            sleep(300);
            //robot.getDriveSubsystem().rotateToShootingAngle();
            //double targetAngle = Math.toDegrees(Math.atan2((robot.getDriveSubsystem().getDataFusionX() - 28.5), (108 - robot.getDriveSubsystem().getDataFusionY()))) - 10;
            while(robot.getDriveSubsystem().getDataFusionTheta() > -18.89) {
                robot.getDriveSubsystem().setRotateMotors(-0.25);
                robot.loop(telemetry);
            }
            robot.getDriveSubsystem().setDriveMotors(0);
            robot.getIOSubsystem().outtakeFromPoint3(Math.hypot(
                    108 - robot.getDriveSubsystem().getDataFusionY(),
                    Math.abs(robot.getDriveSubsystem().getDataFusionX() - 36)));
            sleep(2000);
            robot.getIOSubsystem().runTransfer(true);
            sleep(3000);
            robot.getIOSubsystem().runTransfer(false);
            robot.getIOSubsystem().runOuttake(false);
        }
        double currentPositionY = robot.getDriveSubsystem().getDataFusionY();
        while(robot.getDriveSubsystem().getDataFusionY() < currentPositionY + 12) {
            robot.getDriveSubsystem().setDriveMotors(0.5);
            robot.loop(telemetry);
        }
        robot.getDriveSubsystem().setDriveMotors(0);
        sleep(300);
    }
}