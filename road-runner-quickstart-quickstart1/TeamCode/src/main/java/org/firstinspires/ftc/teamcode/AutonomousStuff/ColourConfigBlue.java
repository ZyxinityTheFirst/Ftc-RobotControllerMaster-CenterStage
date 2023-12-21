package org.firstinspires.ftc.teamcode.AutonomousStuff;

import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueLowerCb;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueLowerCr;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueLowerY;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueUpperCb;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueUpperCr;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.blueUpperY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants.ColourConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
@Autonomous(name = "ConfigColoursBlue")
public class ColourConfigBlue extends LinearOpMode {
    private OpenCvWebcam webcam;
    private ColourConfigBlue.QuadrantPipelineDeterminationBlue12 pipeline1;
    private ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12 snapshotAnalysisBlue = QuadrantPipelineDeterminationBlue12.QuadrantBlue12.ONE;
    private static Scalar blueLowerBound = new Scalar(blueLowerY, blueLowerCr, blueLowerCb);
    private static Scalar blueUpperBound = new Scalar(blueUpperY, blueUpperCr, blueUpperCb);
    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline1 = new ColourConfigBlue.QuadrantPipelineDeterminationBlue12();
        webcam.setPipeline(pipeline1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        waitForStart();

        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            int currentLowerY = ColourConstants.blueLowerY;
            int currentLowerCr = ColourConstants.blueLowerCr;
            int currentLowerCb = ColourConstants.blueLowerCb;
            int currentUpperY = ColourConstants.blueUpperY;
            int currentUpperCr = ColourConstants.blueUpperCr;
            int currentUpperCb = ColourConstants.blueUpperCb;

            blueLowerBound.val[0] = currentLowerY;
            blueLowerBound.val[1] = currentLowerCr;
            blueLowerBound.val[2] = currentLowerCb;
            blueUpperBound.val[0] = currentUpperY;
            blueUpperBound.val[1] = currentUpperCr;
            blueUpperBound.val[2] = currentUpperCb;

            telemetry.addData("Current Quadrant: ", pipeline1.getQuadrantBlue12());
            telemetry.addData("Current center X: ", pipeline1.getCenterBlueX());
            telemetry.update();

            snapshotAnalysisBlue = pipeline1.getQuadrantBlue12();
            FtcDashboard.getInstance().updateConfig();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

    public static class QuadrantPipelineDeterminationBlue12 extends OpenCvPipeline {
        private Rect largestQuadrantRect = null;
        private double maxQuadrantArea = 0;

        public volatile ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12 bluePosition = ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12.ONE;
        double centerBlueX = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat YCrCB = new Mat();
            Imgproc.cvtColor(input, YCrCB, Imgproc.COLOR_RGB2YCrCb);

            Mat isolatedImage = new Mat();
            Core.inRange(YCrCB, blueLowerBound, blueUpperBound, isolatedImage);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(isolatedImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            int width = input.width();
            int height = input.height();
            int quadrantWidth = width / 3;

            for (int i = 0; i < 3; i++) {
                int xStart = i * quadrantWidth;
                int xEnd = (i + 1) * quadrantWidth;
                Rect quadrantRect = new Rect(xStart, 0, quadrantWidth, height);

                List<MatOfPoint> quadrantContours = new ArrayList<>();
                for (MatOfPoint contour : contours) {
                    Rect boundingRect = Imgproc.boundingRect(contour);
                    Point center = new Point(boundingRect.x + (double) boundingRect.width / 2, boundingRect.y + (double) boundingRect.height / 2);
                    if (center.x >= xStart && center.x < xEnd) {
                        quadrantContours.add(contour);
                    }
                    centerBlueX = center.x;
                }

                double quadrantArea = 0;
                for (MatOfPoint contour : quadrantContours) {
                    double area = Imgproc.contourArea(contour);
                    quadrantArea += area;
                }

                if (quadrantArea > maxQuadrantArea) {
                    maxQuadrantArea = quadrantArea;
                    largestQuadrantRect = quadrantRect;
                }
            }

            if (largestQuadrantRect != null) {
                Imgproc.rectangle(isolatedImage, new Point(largestQuadrantRect.x, largestQuadrantRect.y),
                        new Point(largestQuadrantRect.x + largestQuadrantRect.width, largestQuadrantRect.y + largestQuadrantRect.height),
                        new Scalar(0, 0, 255), 2);

                if (centerBlueX < 200) {
                    bluePosition = ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12.ONE;
                } else if (centerBlueX > 200 && centerBlueX < 400) {
                    bluePosition = ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12.TWO;
                } else {
                    bluePosition = ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12.THREE;
                }
            }

            return isolatedImage;
        }

        public ColourConfigBlue.QuadrantPipelineDeterminationBlue12.QuadrantBlue12 getQuadrantBlue12() {
            return bluePosition;
        }

        public double getCenterBlueX(){
            return centerBlueX;
        }

        public int getBlueX() {
            if (largestQuadrantRect != null) {
                return largestQuadrantRect.x;
            }
            return -1;
        }
        public int getBlueY(){
            if (largestQuadrantRect != null){
                return  largestQuadrantRect.y;
            }
            return -1;
        }

        public enum QuadrantBlue12 {
            ONE,
            TWO,
            THREE
        }
    }
}
