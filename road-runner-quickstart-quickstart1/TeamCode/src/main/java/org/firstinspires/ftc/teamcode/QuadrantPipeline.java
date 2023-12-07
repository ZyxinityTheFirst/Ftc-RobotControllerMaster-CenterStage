package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "CameraTest")
public class QuadrantPipeline extends LinearOpMode {

    private OpenCvInternalCamera webCam;
    private QuadrantPipelineDetermination pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new QuadrantPipelineDetermination();
        webCam.setPipeline(pipeline);

        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
            }
        });

        telemetry.addData("The quadrant: ", pipeline.getQuadrant());
        telemetry.addData("idk: ", pipeline.position);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class QuadrantPipelineDetermination extends OpenCvPipeline {
        private Rect largestQuadrantRect = null;
        private double maxQuadrantArea = 0;

        // Adjusted lower and upper bounds for YCbCr: RED
        private static final Scalar lowerBound = new Scalar(0, 110, 165);
        private static final Scalar upperBound = new Scalar(200, 255, 255);

        public volatile Quadrant position = Quadrant.ONE;

        @Override
        public Mat processFrame(Mat input) {
            // Convert the input frame to YCrCb color space
            Mat YCrCB = new Mat();
            Imgproc.cvtColor(input, YCrCB, Imgproc.COLOR_RGB2YCrCb);

            // Apply the color range filter to isolate the desired color (Red)
            Mat isolatedImage = new Mat();
            Core.inRange(YCrCB, lowerBound, upperBound, isolatedImage);

            // Find contours in the isolated image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(isolatedImage, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the quadrant with the largest contour
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

            // Draw a blue rectangle around the largest contour in the largest quadrant
            if (largestQuadrantRect != null) {
                Imgproc.rectangle(input, new Point(largestQuadrantRect.x, largestQuadrantRect.y),
                        new Point(largestQuadrantRect.x + largestQuadrantRect.width, largestQuadrantRect.y + largestQuadrantRect.height),
                        new Scalar(0, 0, 255), 2);

                // Determine the current quadrant
                if (largestQuadrantRect.x == 0) {
                    position = Quadrant.ONE;
                } else if (largestQuadrantRect.x == (input.width() / 3)) {
                    position = Quadrant.TWO;
                } else {
                    position = Quadrant.THREE;
                }
            }

            return input;
        }

        public Quadrant getQuadrant() {
            return position;
        }

        public int getX() {
            if (largestQuadrantRect != null) {
                return largestQuadrantRect.x;
            }
            return -1; // Return a default value when no quadrant is found
        }

        public enum Quadrant {
            ONE,
            TWO,
            THREE
        }
    }
}
