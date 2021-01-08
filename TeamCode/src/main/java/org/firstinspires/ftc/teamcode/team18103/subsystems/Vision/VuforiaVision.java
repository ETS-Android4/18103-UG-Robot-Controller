package org.firstinspires.ftc.teamcode.team18103.subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.halfField;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.mmTargetHeight;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.quadField;

/*
 * Author: Akhil G
 */

public class VuforiaVision extends Subsystem {

    private VuforiaLocalizer vuforia;
    private WebcamName webcamName;

    private OpenGLMatrix lastKnownLocation = MathFx.createMatrix(0, 0, 0, 0, 0, 0);

    private float X = 0;
    private float Y = 0;
    private float Theta = 0;

    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void init(HardwareMap ahMap) {

        webcamName = ahMap.get(WebcamName.class, Constants.webcamName);

        int cameraMonitorViewId = ahMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", ahMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initVuforia();

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(Constants.robotFromCamera, parameters.cameraDirection);
        }
    }

    @Override
    public void start() {

    }

    public VuforiaTrackable search(VuforiaTrackable target) {
        if (((VuforiaTrackableDefaultListener)target.getListener()).isVisible()) {
            return target;
        } else {
            return null;
        }
    }

    public List<VuforiaTrackable> search() {
        List<VuforiaTrackable> visible = new ArrayList<VuforiaTrackable>();
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                visible.add(trackable);
            }
        }
        return visible;
    }

    public void getRobotLocation() {
        boolean targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (search(trackable) != null) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }
                VectorF translation = lastKnownLocation.getTranslation();
                X = translation.get(0)/Constants.mmPerInch;
                Y = translation.get(1)/Constants.mmPerInch;
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastKnownLocation, EXTRINSIC, XYZ, DEGREES);
                Theta = rotation.thirdAngle;
            }
            break;
        }
    }

    @Override
    public void update() {

        //getRobotLocation();
    }

    private void initVuforia() {
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables.addAll(targetsSkyStone);
        // Setting Traceable Target Locations
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
    }

    public float getX() {
        //getRobotLocation();
        return X;
    }

    public float getY() {
        //getRobotLocation();
        return Y;
    }

    public float getTheta() {
        //getRobotLocation();
        return Theta;
    }

}
