package org.firstinspires.ftc.teamcode.team18103.subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;

import java.util.List;

import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_FIRST_ELEMENT;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.LABEL_SECOND_ELEMENT;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.TFOD_MODEL_ASSET;
import static org.firstinspires.ftc.teamcode.team18103.src.Constants.VUFORIA_KEY;

/*
 * Author: Akhil G
 */

public class TFVision extends Subsystem {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private List<Recognition> updatedRecognitions;

    @Override
    public void init(HardwareMap ahMap) {
        initVuforia();
        initTfod(ahMap);
        tfod.activate();
    }

    @Override
    public void start() {
        search();
    }

    @Override
    public void update() {

    }

    private List<Recognition> search() {
        updatedRecognitions = tfod.getUpdatedRecognitions();
        return updatedRecognitions;
    }

    public AutoMode getAutoMode() {
        for (Recognition i : search()) {
            if (i.getLabel() == LABEL_FIRST_ELEMENT) {
                return AutoMode.FourRings;
            } else if (i.getLabel() == LABEL_SECOND_ELEMENT) {
                return AutoMode.OneRing;
            }
        }
        return AutoMode.NoRings;
    }

    private void initVuforia() {
       VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
       parameters.vuforiaLicenseKey = VUFORIA_KEY;
       parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
       vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
