package org.firstinspires.ftc.teamcode.team18103.subsystems.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;
import org.firstinspires.ftc.teamcode.team18103.states.AutoMode;
import org.firstinspires.ftc.teamcode.team18103.subsystems.Subsystem;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class EOCVision extends Subsystem {

    OpenCvCamera webcam;
    UGDeterminationPipeline pipeline;

    @Override
    public void init(HardwareMap ahMap) {
        int cameraMonitorViewId = ahMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(ahMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new UGDeterminationPipeline();
        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    public static class UGDeterminationPipeline extends OpenCvPipeline {

        Point region1_pointA = new Point(
                Constants.REGION1_TOP_LEFT_ANCHOR_POINT.x,
                Constants.REGION1_TOP_LEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                Constants.REGION1_TOP_LEFT_ANCHOR_POINT.x + Constants.REGION_WIDTH,
                Constants.REGION1_TOP_LEFT_ANCHOR_POINT.y + Constants.REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile AutoMode position = AutoMode.Four;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    Constants.BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = AutoMode.Four; // Record our analysis
            if(avg1 > Constants.FOUR_RING_THRESHOLD){
                position = AutoMode.Four;
            }else if (avg1 > Constants.ONE_RING_THRESHOLD){
                position = AutoMode.One;
            }else{
                position = AutoMode.None;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    Constants.GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

    }

    public AutoMode getAutoMode() {
        return pipeline.position;
    }

}
