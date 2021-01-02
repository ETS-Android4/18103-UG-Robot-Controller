package org.firstinspires.ftc.teamcode.team18103.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumKinematicEstimator;
import org.firstinspires.ftc.teamcode.team18103.states.GameState;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Drive1;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake.Intake;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake.Outtake;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.TriWheelOdometryGPS;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IntakeOuttake.Transfer;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Wobble;

/*
 * Author: Akhil G
 */

public class Robot {
    ElapsedTime elapsedTime = new ElapsedTime();
    GameState gameState;

    Subsystem[] subsystems;

    //TriWheelOdometryGPS odometry = new TriWheelOdometryGPS(Motor.REV_Encoder.getTicksPerInch(35), Constants.dt);
    //MecanumKinematicEstimator MKEstimator = new MecanumKinematicEstimator();
    Drive1 DriveSubsystem = new Drive1();
    Intake IntakeSubsystem = new Intake();
    Transfer TransferSubsystem = new Transfer();
    Outtake OuttakeSubsystem = new Outtake();
    //Wobble WobbleSubsystem = new Wobble();

    public Robot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        subsystems = new Subsystem[]{DriveSubsystem, IntakeSubsystem,
                TransferSubsystem, OuttakeSubsystem /*WobbleSubsystem*/};

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
        telemetry.addLine()
                .addData("Robot Initialized: ", true);

        // Game State
//        telemetry.addLine()
//                .addData("Game State: ", getGameState().getName());
//
        // IMU Measurements
        //telemetry.addLine()
        //        .addData("IMU Roll: ", getImu().getRoll())
        //        .addData("IMU Pitch: ", getImu().getPitch())
        //        .addData("IMU Heading: ", getImu().getHeading());
        // Odometry
        telemetry.addLine()
                .addData("Odometry X: ", getDriveSubsystem().getOdometry().getX())
                .addData("Odometry Y: ", getDriveSubsystem().getOdometry().getY())
                .addData("Odometry Theta: ", getDriveSubsystem().getOdometry().getTheta());
        // Vision
        /*telemetry.addLine()
                .addData("Vision X: ", getVision().getX())
                .addData("Vision Y: ", getVision().getY())
                .addData("Vision Theta: ", getVision().getTheta());
        */
        // Data Fusion Model
        telemetry.addLine()
                .addData("Data Fusion X: ", getDriveSubsystem().getDataFusionX())
                .addData("Data Fusion Y: ", getDriveSubsystem().getDataFusionY())
                .addData("Data Fusion Theta: ", getDriveSubsystem().getDataFusionTheta());
        // Collision Detection
//        telemetry.addLine()
//                .addData("Collision Detected: ", getImu().getCollision());
//        // Drive Mode
//        telemetry.addLine()
//                .addData("Drive Mode: ", getDriveSubsystem().getDriveMode().getName());

        telemetry.addLine().addData("First Outtake RPM: ", getOuttakeSubsystem().getFirstOuttakeRPM());
        telemetry.addLine().addData("Second Outtake RPM: ", getOuttakeSubsystem().getSecondOuttakeRPM());
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

    public Drive1 getDriveSubsystem() {
        return DriveSubsystem;
    }

    public Intake getIntakeSubsystem() {
        return IntakeSubsystem;
    }

    public Outtake getOuttakeSubsystem() {
        return OuttakeSubsystem;
    }

    public Transfer getTransferSubsystem() {
        return TransferSubsystem;
    }

    public Wobble getWobbleSubsystem() {
        return null; //WobbleSubsystem;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }

    //public VuforiaVision getVuforiaVision() {
    //    return vuforiaVision;
    //}

}
