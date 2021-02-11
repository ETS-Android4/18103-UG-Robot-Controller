package org.firstinspires.ftc.teamcode.team18103.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.control.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.ProfileState;
import org.firstinspires.ftc.teamcode.lib.motion.TrapezoidalMotionProfileGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumKinematicEstimator;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.lib.util.MeanOptimizedDataFusionModel;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.states.DriveMode;
import org.firstinspires.ftc.teamcode.team18103.subsystems.IMU.REV_IMU;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Odometry.TriWheelOdometryGPS;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.EOCVision;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.TFVision;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Vision.VuforiaVision;

import java.util.Arrays;

public class Drive extends Subsystem {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotorEx[] driveMotors;

    REV_IMU imu;
    //TriWheelOdometryGPS odometry;
    //VuforiaVision visualOdometry;
    //TFVision visionProcessing;
    EOCVision visionProcessing;
    MecanumKinematicEstimator MKEstimator;
    MeanOptimizedDataFusionModel ThetaModel;
    MeanOptimizedDataFusionModel XModel;
    MeanOptimizedDataFusionModel YModel;

    DriveMode driveMode = DriveMode.Balanced;
    int driveType = 1; // 0 - Field-Centric, 1 - POV
    ProfileState driveState;
    double x, y, theta;

    public Drive() {
        ThetaModel = new MeanOptimizedDataFusionModel();
        YModel = new MeanOptimizedDataFusionModel();
        XModel = new MeanOptimizedDataFusionModel();
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
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // IMU Init
        imu = new REV_IMU();
        imu.init(ahMap);

        // Odometry Init
        //odometry = new TriWheelOdometryGPS();
        //odometry.init(ahMap);

        /* Visual Odometry Init */
        //visualOdometry = new VuforiaVision();
        //visualOdometry.init(ahMap);

        /* Vision Processing Init */
        //visionProcessing = new TFVision();
        visionProcessing = new EOCVision();
        visionProcessing.init(ahMap);//*/

        // MKE Init
        MKEstimator = new MecanumKinematicEstimator();
        MKEstimator.init(ahMap);
    }

    @Override
    public void start() {
        imu.start();
        //odometry.start();
        //visualOdometry.start();
        visionProcessing.start();
        MKEstimator.start();
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

    public void MecanumDriveToGlobalPoint(double y, double x, double theta) {

        double maxY = y;
        double maxX = x;

        while ((Math.abs(y - getDataFusionY()) > 10) &&
                (Math.abs(x - getDataFusionX()) > 10) &&
                (Math.abs(theta - getDataFusionTheta()) > 10)) {
            POVMecanumDrive(y/maxY, x/maxX, theta/360, DriveMode.Balanced);
        }
        setDriveMotors(0);
    }

    public void MecanumDriveSraightToGlobalPoint(double y, double power) {

        double maxVel = y - getDataFusionY();

        while ((Math.abs(y - getDataFusionY()) > 30)) {
            //frontLeft.setVelocity(power);
            //backLeft.setVelocity();
            //backRight.setVelocity();
            //frontRight.setVelocity();

        }

        setDriveMotors(0);
    }

    public double rotateToShootingAngle() {
        //double targetTheta = Math.toDegrees(Math.atan2(Constants.Gx-getDataFusionX(),
        //        Constants.Gy-getDataFusionY())) - 10;

        double targetTheta = Math.toDegrees(Math.atan((Constants.Gx-getDataFusionX())/(Constants.Gy-getDataFusionY()))) - 10;

        boolean left = true;
        if (targetTheta > getDataFusionTheta()) {
            left = false;
        }

        while (left && getDataFusionTheta() > targetTheta) {
            setRotateMotors(-0.5);
        }

        while (!left && getDataFusionTheta() < targetTheta) {
            setRotateMotors(0.5);
        }

        //CustomDriveRotate(targetTheta, 15);

        return targetTheta;
    }

    public void CustomDriveRotate(double targetAngle,
                                  double tolerance) {
        double dist = Math.abs(targetAngle
                - getDataFusionTheta());
        while (Math.abs(targetAngle -
                getDataFusionTheta()) > tolerance) {
            POVMecanumDrive(0, 0,
                    (targetAngle -
                            getDataFusionTheta())/dist + 0.25,
                    DriveMode.Balanced);
        }
        setDriveMotors(0);
    }

    public void CustomDriveStraight(double distance, double tolerance) {
        while (Math.abs(distance - getDataFusionY()) > 15) {
            POVMecanumDrive(-1 + (getDataFusionY()) / distance, 0, 0, DriveMode.Balanced);
        }
        setDriveMotors(0);
    }

    public void CustomDriveStrafe(double distance, double tolerance) {
        while (Math.abs(distance - getDataFusionX()) > 15) {
            POVMecanumDrive(0, 1 + (getDataFusionX()) / distance, 0, DriveMode.Balanced);
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

    public void zeroCoords(boolean button) {
        if (button) {
            zeroYaw();
            zeroX();
            zeroY();
        }
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
        imu.update();
        //odometry.update();
        //visualOdometry.update();
        visionProcessing.update();
        MKEstimator.update();
        getDataFusionX();
        getDataFusionY();
        getDataFusionTheta();
    }

    public void zeroYaw() {
        ThetaModel.setBias(-getDataFusionTheta());
    }

    public void zeroX() {
        XModel.setBias(-getDataFusionX());
    }

    public void zeroY() {
        YModel.setBias(-getDataFusionY());
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
        return null; //odometry;
    }

    public VuforiaVision getVisualOdometry() {
        return null; //visualOdometry;
    }

    public MecanumKinematicEstimator getMKEstimator() {
        return MKEstimator;
    }

    public REV_IMU getImu() {
        return imu;
    }

    public double getDataFusionTheta() {
        setTheta(ThetaModel.fuse(new double[]{/*imu.getHeading(), odometry.getTheta(),
                /*visualOdometry.getTheta(),*/ MKEstimator.getTheta()}));
        return theta;
    }

    public double getDataFusionX() {
        setX(XModel.fuse(new double[]{/*odometry.getX(),*/ /*visualOdometry.getX(),*/ MKEstimator.getX()}));
        return x;
    }

    public double getDataFusionY() {
        setY(YModel.fuse(new double[]{/*odometry.getY(),*/ /*visualOdometry.getY(),*/ MKEstimator.getY()}));
        return y;
    }

    public void setDriveState(ProfileState driveState) {
        this.driveState = driveState;
    }

    public ProfileState getDriveState() {
        return driveState;
    }

    public MeanOptimizedDataFusionModel getThetaModel() {
        return ThetaModel;
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

    public /*TFVision*/ EOCVision getVisionProcessing() {
        return visionProcessing;
    }
}
