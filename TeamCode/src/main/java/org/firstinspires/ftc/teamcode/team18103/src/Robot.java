package org.firstinspires.ftc.teamcode.team18103.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.team18103.states.GameState;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake.IntakeOuttake;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Wobble;

import java.text.DecimalFormat;

/*
 * Author: Akhil G
 */

public class Robot {
    ElapsedTime elapsedTime = new ElapsedTime();
    GameState gameState;
    DecimalFormat df = new DecimalFormat("#.##");

    Subsystem[] subsystems;

    Drive DriveSubsystem = new Drive();
    IntakeOuttake IOSubsystem = new IntakeOuttake();
    //Wobble WobbleSubsystem = new Wobble();

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        subsystems = new Subsystem[]{DriveSubsystem, IOSubsystem, /*WobbleSubsystem*/};

        for (Subsystem subsystem : subsystems) {
            subsystem.init(hardwareMap);
        }

        setGameState(GameState.TeleOp);

        resetElapsedTime();

        Telemetry(telemetry);
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void loop(Telemetry telemetry) {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }

        Telemetry(telemetry);
    }

    public void Telemetry(Telemetry telemetry) {
        // Robot Init
//        telemetry.addLine()
//                .addData("Robot Initialized: ", true);

        /* Game State
        telemetry.addLine()
                .addData("Game State: ", getGameState().getName());*/

        /* IMU Measurements
        telemetry.addLine()
                .addData("IMU Roll: ", getDriveSubsystem().getImu().getRoll())
                .addData("IMU Pitch: ", getDriveSubsystem().getImu().getPitch())
                .addData("IMU Heading: ", getDriveSubsystem().getImu().getHeading());//*/

        /* Odometry
        telemetry.addLine()
                .addData("Odo X: ", df.format(getDriveSubsystem().getOdometry().getX()))
                .addData("Odo Y: ", df.format(getDriveSubsystem().getOdometry().getY()))
                .addData("Odo Theta: ", df.format(getDriveSubsystem().getOdometry().getTheta()));//*/

        /* Odometry Debugging
        telemetry.addLine()
                .addData("Left: ", df.format(getDriveSubsystem().getOdometry().getlPos()))
                .addData("Right: ", df.format(getDriveSubsystem().getOdometry().getrPos()))
                .addData("Horz: ", df.format(getDriveSubsystem().getOdometry().getsPos()));//*/

        /* MKE */
        telemetry.addLine()
                .addData("MKE X: ", df.format(getDriveSubsystem().getMKEstimator().getX()))
                .addData("MKE Y: ", df.format(getDriveSubsystem().getMKEstimator().getY()))
                .addData("MKE Theta: ", df.format(getDriveSubsystem().getMKEstimator().getTheta()));//*/
//
//        /* Visual Odometry
//        telemetry.addLine()
//                .addData("Vision X: ", getDriveSubsystem().getVisualOdometry().getX())
//                .addData("Vision Y: ", getDriveSubsystem().getVisualOdometry().getY())
//                .addData("Vision Theta: ", getDriveSubsystem().getVisualOdometry().getTheta());*/
//
//        /* Data Fusion Model */
//        telemetry.addLine()
//                .addData("Data Fusion X: ", df.format(getDriveSubsystem().getDataFusionX()))
//                .addData("Data Fusion Y: ", df.format(getDriveSubsystem().getDataFusionY()))
//                .addData("Data Fusion Theta: ", df.format(getDriveSubsystem().getDataFusionTheta())); //*/

        /* Collision Detection
        telemetry.addLine()
                .addData("Collision Detected: ", getDriveSubsystem().getImu().getCollision());*/

        /* Drive Mode
        telemetry.addLine()
                .addData("Drive Mode: ", getDriveSubsystem().getDriveMode().getName());*/

        /* Outtake
        telemetry.addLine()
                .addData("Shooter 1 RPM: ", getIOSubsystem().shooterDiagnostics()[0])
                .addData("SShooter 2 RPM: ", getIOSubsystem().shooterDiagnostics()[1]);*/

        telemetry.update();

    }

    public GameState getGameState() {
        if (elapsedTime.seconds() < GameState.TeleOp.getEndTime()) {
            setGameState(GameState.TeleOp);
        } else {
            setGameState(GameState.EndGame);
        }
        return gameState;
    }

    public void resetElapsedTime() {
        elapsedTime.reset();
    }

    public Drive getDriveSubsystem() {
        return DriveSubsystem;
    }

    public IntakeOuttake getIOSubsystem() {
        return IOSubsystem;
    }

    public Wobble getWobbleSubsystem() {
        return null; //WobbleSubsystem;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }

}