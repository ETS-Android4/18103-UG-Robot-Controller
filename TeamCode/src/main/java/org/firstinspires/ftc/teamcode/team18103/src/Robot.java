package org.firstinspires.ftc.teamcode.team18103.src;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumKinematicEstimator;
import org.firstinspires.ftc.teamcode.team18103.states.GameState;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.TriWheelOdometryGPS;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.TFVision;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.VuforiaVision;

/*
 * Author: Akhil G
 */

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private GameState gameState;

    private Subsystem[] subsystems;

    private REV_IMU imu = new REV_IMU();
    private TriWheelOdometryGPS odometry = new TriWheelOdometryGPS(Motor.GoBILDA_312.getTicksPerInch(), Constants.dt);
    private VuforiaVision vuforiaVision = new VuforiaVision();
    private MecanumKinematicEstimator MKEstimator = new MecanumKinematicEstimator();
    private TFVision tfVision = new TFVision();
    private Drive DriveSubsystem = new Drive(imu, odometry, vuforiaVision, MKEstimator);

    public Robot(HardwareMap hMap, Telemetry tele) {
        hardwareMap = hMap;
        telemetry = tele;
    }

    public void init() {
        subsystems = new Subsystem[]{DriveSubsystem, imu, vuforiaVision, odometry, MKEstimator, tfVision};

        for (Subsystem subsystem : subsystems) {
            subsystem.init(hardwareMap);
        }

        setGameState(GameState.TeleOp);

        resetElapsedTime();

        Telemetry();
    }

    public void start() {
        for (Subsystem subsystem : subsystems) {
            subsystem.start();
        }
    }

    public void loop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }

        getGameState();

        Telemetry();
    }

    public void Telemetry() {
        // Robot Init
        telemetry.addLine()
                .addData("Robot Initialized: ", true);
        // Game State
        telemetry.addLine()
                .addData("Game State: ", getGameState().getName());
        /*
        // IMU Measurements
        telemetry.addLine()
                .addData("IMU Roll: ", getImu().getRoll())
                .addData("IMU Pitch: ", getImu().getPitch())
                .addData("IMU Heading: ", getImu().getHeading());
        // Odometry
        telemetry.addLine()
                .addData("Odometry X: ", getOdometry().getX())
                .addData("Odometry Y: ", getOdometry().getY())
                .addData("Odometry Theta: ", getOdometry().getTheta());
        // Vision
        telemetry.addLine()
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
        telemetry.addLine()
                .addData("Collision Detected: ", getImu().getCollision());
        // Drive Mode
        telemetry.addLine()
                .addData("Drive Mode: ", getDriveSubsystem().getDriveMode().getName());

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

    public REV_IMU getImu() {
        return imu;
    }

    public void setGameState(GameState gameState) {
        this.gameState = gameState;
    }

    public TriWheelOdometryGPS getOdometry() {
        return odometry;
    }

    public VuforiaVision getVuforiaVision() {
        return vuforiaVision;
    }

}
