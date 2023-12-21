package org.firstinspires.ftc.teamcode.AutonomousStuff;

import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.lowerCb;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.lowerCr;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.lowerY;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.upperCb;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.upperCr;
import static org.firstinspires.ftc.teamcode.Constants.ColourConstants.upperY;

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

@Autonomous(name = "ConfigColours")
public class ColourConfig extends LinearOpMode {

    private OpenCvWebcam webcam;
    private QuadrantPipelineDetermination12 pipeline1;

    private static Scalar lowerBound = new Scalar(lowerY, lowerCr, lowerCb);
    private static Scalar upperBound = new Scalar(upperY, upperCr, upperCb);

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);        pipeline1 = new ColourConfig.QuadrantPipelineDetermination12();
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

            int currentLowerY = ColourConstants.lowerY;
            int currentLowerCr = ColourConstants.lowerCr;
            int currentLowerCb = ColourConstants.lowerCb;
            int currentUpperY = ColourConstants.upperY;
            int currentUpperCr = ColourConstants.upperCr;
            int currentUpperCb = ColourConstants.upperCb;

            lowerBound.val[0] = currentLowerY;
            lowerBound.val[1] = currentLowerCr;
            lowerBound.val[2] = currentLowerCb;
            upperBound.val[0] = currentUpperY;
            upperBound.val[1] = currentUpperCr;
            upperBound.val[2] = currentUpperCb;

            telemetry.addData("Current Quadrant: ", pipeline1.getQuadrant12());
            telemetry.addData("Current center X: ", pipeline1.getCenterX());
            telemetry.update();

            FtcDashboard.getInstance().updateConfig();
        }

        FtcDashboard.getInstance().stopCameraStream();
    }

    public static class QuadrantPipelineDetermination12 extends OpenCvPipeline {
        private Rect largestQuadrantRect = null;
        private double maxQuadrantArea = 0;

        public volatile Quadrant12 position = Quadrant12.ONE;
        double centerX = 0;

        @Override
        public Mat processFrame(Mat input) {
            Mat YCrCB = new Mat();
            Imgproc.cvtColor(input, YCrCB, Imgproc.COLOR_RGB2YCrCb);

            Mat isolatedImage = new Mat();
            Core.inRange(YCrCB, lowerBound, upperBound, isolatedImage);

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
                    centerX = center.x;
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
                Imgproc.rectangle(input, new Point(largestQuadrantRect.x, largestQuadrantRect.y),
                        new Point(largestQuadrantRect.x + largestQuadrantRect.width, largestQuadrantRect.y + largestQuadrantRect.height),
                        new Scalar(0, 0, 255), 2);

                if (centerX < 200) {
                    position = Quadrant12.ONE;
                } else if (centerX > 200 && centerX < 400) {
                    position = Quadrant12.TWO;
                } else {
                    position = Quadrant12.THREE;
                }
            }

            return isolatedImage;
        }

        public Quadrant12 getQuadrant12() {
            return position;
        }

        public double getCenterX(){
            return centerX;
        }

        public int getX() {
            if (largestQuadrantRect != null) {
                return largestQuadrantRect.x;
            }
            return -1;
        }
        public int getY(){
            if (largestQuadrantRect != null){
                return  largestQuadrantRect.y;
            }
            return -1;
        }

        public enum Quadrant12 {
            ONE,
            TWO,
            THREE
        }
    }
}
