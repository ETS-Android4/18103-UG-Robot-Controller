package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.ProfileState;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumKinematicEstimator;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.lib.util.MeanOptimizedDataFusionModel;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.TriWheelOdometryGPS;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.VuforiaVision;

import java.util.Arrays;

public class Drive extends Subsystem {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx[] driveMotors;

    REV_IMU imu;
    TriWheelOdometryGPS odometry;
    VuforiaVision vision;
    MecanumKinematicEstimator MKEstimator;
    MeanOptimizedDataFusionModel model;

    DriveMode driveMode = DriveMode.Balanced;
    int driveType = 1; // 0 - Field-Centric, 1 - POV
    ProfileState driveState;
    double x, y, theta;

    public Drive() {
        model = new MeanOptimizedDataFusionModel();
    }

    public Drive(DcMotor Left, DcMotor Right, DcMotor Horizontal) {
        model = new MeanOptimizedDataFusionModel();
        this.odometry = new TriWheelOdometryGPS(Left, Right, Horizontal,
                Motor.REV_Encoder.getTicksPerInch(35), Constants.dt);
    }

    @Override
    public void init(HardwareMap ahMap) {
        // Drive Init
        frontLeft = ahMap.get(DcMotorEx.class, Constants.frontLeft);
        frontRight = ahMap.get(DcMotorEx.class, Constants.frontRight);
        backLeft = ahMap.get(DcMotorEx.class, Constants.backLeft);
        backRight = ahMap.get(DcMotorEx.class, Constants.backRight);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        for (DcMotorEx motor : driveMotors) {
            //motor.setPositionPIDFCoefficients(Constants.DRIVE_P);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // IMU Init
        imu = new REV_IMU();
        imu.init(ahMap);
        // Odometry Init

        // Vision Init

        // MKE Init

    }

    public void setOdometry(HardwareMap ahMap, DcMotor Left, DcMotor Right, DcMotor Horizontal) {
        odometry = new TriWheelOdometryGPS(Left, Right, Horizontal,
                Motor.REV_Encoder.getTicksPerInch(35), Constants.dt);

        odometry.init(ahMap);
    }

    @Override
    public void start() {

    }

    // Autonomous Algorithms

    // Setting Drivetrain

    /**
     * Sets Drive to go forward/backwards
     * @param power Speed of movement
     */
    public void setDriveMotors(double power) {
        for (DcMotorEx i : driveMotors) {
            i.setPower(power);
        }
    }

    /**
     * Sets Drive to go left/right
     * @param power Speed of movement
     */
    public void setStrafeMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    /**
     * Sets Drive to rotate
     * @param power Speed of Movement
     */
    public void setRotateMotors(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }

    // Distanced Driving (Motion Profiling)

    /**
     * Sets Drive to rotate to a certain angle (-180, 180) w/ motion profile & PIDSVA
     * @param heading rotation angle
     */
    public void PointRotation(double heading) {
        double curHead = getDataFusionTheta();

        double error = heading - curHead;

        double distance = Constants.ENCODER_DIFFERENCE * Math.PI * error / 360;

        TrapezoidalMotionProfileGenerator motionProfile = new TrapezoidalMotionProfileGenerator(distance, Motor.GoBILDA_312);
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime();
        PIDSVA controller = new PIDSVA(10d/12d, 1d/12d, 10d/12d, 0d, 1d/motionProfile.getMaxV(), 0.01d/12d);
        while (timer.seconds() < motionProfile.getTotalTime()) {
            double timeStamp = timer.seconds();
            double position = motionProfile.getPosition(timeStamp);
            double velocity = motionProfile.getVelocity(timeStamp);
            double acceleration = motionProfile.getAcceleration(timeStamp);
            double pos_error = position - (frontLeft.getCurrentPosition()/ Motor.GoBILDA_312.getTicksPerInch());
            double output = controller.getOutput(pos_error, velocity, acceleration);

            setRotateMotors(output);

            setDriveState(motionProfile.getProfileState(timeStamp));

        }
        setDriveMotors(0);
    }

    /**
     * Sets Drive to go forward/backwards w/ motion profile & PIDSVA
     * @param distance target setpoint
     */
    public void motionProfileDrive(double distance) {
        TrapezoidalMotionProfileGenerator motionProfile = new TrapezoidalMotionProfileGenerator(distance, Motor.GoBILDA_312);
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime();
        PIDSVA controller = new PIDSVA(0, 0, 0, 0d, 1d/motionProfile.getMaxV(), 0);
        while (timer.seconds() < motionProfile.getTotalTime()) {
            double timeStamp = timer.seconds();
            double position = motionProfile.getPosition(timeStamp);
            double velocity = motionProfile.getVelocity(timeStamp);
            double acceleration = motionProfile.getAcceleration(timeStamp);
            double error = position - (frontLeft.getCurrentPosition()/ Motor.GoBILDA_312.getTicksPerInch());
            double output = controller.getOutput(error, velocity, acceleration);

            setDriveMotors(output);

            setDriveState(motionProfile.getProfileState(timeStamp));

        }
        setDriveMotors(0);
    }

    /**
     * Sets Drive to go left/right w/ motion profile & PIDSVA
     * @param distance target setpoint
     */
    public void motionProfileStrafe(double distance) {
        TrapezoidalMotionProfileGenerator motionProfile = new TrapezoidalMotionProfileGenerator(distance, Motor.GoBILDA_312);
        for (DcMotorEx i : driveMotors) {
            i.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        ElapsedTime timer = new ElapsedTime();
        PIDSVA controller = new PIDSVA(0, 0, 0, 0d, 1d/motionProfile.getMaxV(), 0);
        while (timer.seconds() < motionProfile.getTotalTime()) {
            double timeStamp = timer.seconds();
            double position = motionProfile.getPosition(timeStamp);
            double velocity = motionProfile.getVelocity(timeStamp);
            double acceleration = motionProfile.getAcceleration(timeStamp);
            double error = position - (frontLeft.getCurrentPosition()/ Motor.GoBILDA_312.getTicksPerInch());
            double output = controller.getOutput(error, velocity, acceleration);

            setStrafeMotors(output);

            setDriveState(motionProfile.getProfileState(timeStamp));

        }
        setDriveMotors(0);
    }

    // TeleOp Methods

    /**
     * Tank Drive Control
     * @param left left side power
     * @param right right side power
     */
    public void tankDrive(double left, double right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void POVMecanumDrive(double y, double x, double turn, DriveMode mode) {
        turn *= 0.75; //Custom reduction bc it was requested.
        double v1 = -(y - (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v2 = -(y - (turn * Constants.strafeScaling) + (x/Constants.turnScaling));
        double v3 = -(y + (turn * Constants.strafeScaling) - (x/Constants.turnScaling));
        double v4 = -(y + (turn * Constants.strafeScaling) + (x/Constants.turnScaling));

        Double[] v = new Double[]{Math.abs(v1), Math.abs(v2), Math.abs(v3), Math.abs(v4)};

        Arrays.sort(v);

        if (v[3] > 1) {
            v1 /= v[3];
            v2 /= v[3];
            v3 /= v[3];
            v4 /= v[3];
        }

        frontLeft.setPower(v1 * mode.getScaling());
        backLeft.setPower(v2 * mode.getScaling());
        backRight.setPower(v3 * mode.getScaling());
        frontRight.setPower(v4 * mode.getScaling());
    }

    /**
     * Field-Centric Mecanum Drive Control
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param mode Drivetrain Speed Setting (Sport, Normal, Economy)
     */
    public void fieldCentricMecanumDrive(double y, double x, double turn, DriveMode mode) {
        x = x * Math.cos(imu.getHeading()) - y * Math.sin(imu.getHeading());
        y = x * Math.sin(imu.getHeading()) + y * Math.cos(imu.getHeading());

        POVMecanumDrive(y, x, turn, mode);
    }

    /**
     * Field-Centric Tank Drive Control
     * @param y Forward/Backward Input (GamePad Right Stick y)
     * @param x Left/Right Input (GamePad Right Stick x)
     * @param largeTurn Determination of when the optimal angle exceeds 100 degrees whether to make a large turn (true) or drive backwards (false)
     * @param mode DriveMode Speed Setting (Sport, Normal, Economy)
     */
    public void fieldCentricTankDrive(double y, double x, boolean largeTurn, DriveMode mode) {
        double targetSpeed = Math.max(Math.abs(x), Math.abs(y));
        double targetDirection = Math.atan2(x, y) * (180/Math.PI);

        double angleError = targetDirection - imu.getHeading();

        while(angleError>180)
            angleError-=360;
        while(angleError<180)
            angleError+=360;

        if(!largeTurn) {
            if(angleError > 100) {
                angleError-=180;
                targetSpeed = -targetSpeed;
            } else if(angleError < -100) {
                angleError+=180;
                targetSpeed = -targetSpeed;
            }
        }

        double targetTurn = angleError/45;

        if(Math.abs(targetSpeed) < 0.1)
            targetSpeed = 0;

        double leftSidePower = targetSpeed + targetTurn;
        double rightSidePower = targetSpeed - targetTurn;

        leftSidePower = Math.max(Math.min(1, leftSidePower), -1);
        rightSidePower = Math.max(Math.min(1, rightSidePower), -1);

        frontLeft.setPower(leftSidePower * mode.getScaling());
        backLeft.setPower(leftSidePower * mode.getScaling());
        backRight.setPower(rightSidePower * mode.getScaling());
        frontRight.setPower(rightSidePower * mode.getScaling());
    }

    /**
     * Ultimate Drive Controller
     * @param y Forward/Backward Force (GamePad Left Stick y)
     * @param x Left/Right (Strafe) Force (GamePad Left Stick x)
     * @param turn Rotational Force (GamePad Right Stick x)
     * @param left_trigger Lower Gear Adjustment
     * @param right_trigger Higher Gear Adjustment
     * @param left_bumper Field-Centric Drive Setter
     * @param right_bumper Point-of-View Drive Setter
     * @param button Zero Yaw Setter
     */

    public void ultimateDriveController(double y, double x, double turn, float left_trigger,
                                        float right_trigger, boolean left_bumper,
                                        boolean right_bumper, boolean button) {

        double mode = driveMode.getId();
        double modeChange = right_trigger - left_trigger;
        if (Math.abs(modeChange) > 0.5) {
            mode = MathFx.scale(-1, 1, Math.round(mode + modeChange));
        }

        setDriveMode(getDMbyID(mode));

        if (left_bumper) {
            setDriveType(0);
        }

        if (right_bumper) {
            setDriveType(1);
        }

        if (button) {
            zeroYaw();
        }


        switch (driveType) {
            case 0:
                fieldCentricMecanumDrive(y, x, turn, driveMode);
                break;
            case 1:
                POVMecanumDrive(y, x, turn, driveMode);
                break;
        }

    }

    @Override
    public void update() {
        getDataFusionX();
        getDataFusionY();
        getDataFusionTheta();
    }

    public void zeroYaw() {
        model.setBias(-getDataFusionTheta());
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public DriveMode getDMbyID(double id) {
        if (id == -1) {
            return DriveMode.Economy;
        } else if (id == 0) {
            return DriveMode.Balanced;
        } else {
            return DriveMode.Sport;
        }
    }

    public double getDriveType() {
        return driveType;
    }

    public void setDriveType(int driveType) {
        this.driveType = driveType;
    }

    public TriWheelOdometryGPS getOdometry() {
        return odometry;
    }

    public VuforiaVision getVision() {
        return vision;
    }

    public MecanumKinematicEstimator getMKEstimator() {
        return MKEstimator;
    }

    public REV_IMU getImu() {
        return imu;
    }

    public double getDataFusionTheta() {
        setTheta(model.fuse(new double[]{imu.getHeading(), odometry.getTheta()}));
        return theta;
    }

    public double getDataFusionX() {
        setX(model.fuse(new double[]{odometry.getX()}));
        return x;
    }

    public double getDataFusionY() {
        setY(model.fuse(new double[]{odometry.getY()}));
        return y;
    }

    public void setDriveState(ProfileState driveState) {
        this.driveState = driveState;
    }

    public ProfileState getDriveState() {
        return driveState;
    }

    public MeanOptimizedDataFusionModel getModel() {
        return model;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getTheta() {
        return theta;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }
}
