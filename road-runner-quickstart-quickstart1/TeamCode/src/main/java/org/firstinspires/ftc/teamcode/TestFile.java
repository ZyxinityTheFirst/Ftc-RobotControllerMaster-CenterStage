package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "testFileForAprilTags")
public class TestFile extends LinearOpMode {

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
    double fx = ColourConstants.fx;
//    double fy = 578.272;
    double fy = ColourConstants.fy;
//    double cx = 402.145;
    double cx = ColourConstants.cx;
//    double cy = 221.506;
    double cy = ColourConstants.cy;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;
    AprilTagPipeline.AprilTagDetectionPipeline pipeline;
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //public AprilTagDetectionPipeline(double tagsize, double fx, double fy, double cx, double cy)
        pipeline = new AprilTagPipeline.AprilTagDetectionPipeline(tagsize, fx,fy,cx,cy);
        webcam.setPipeline(pipeline);
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

        FtcDashboard.getInstance().startCameraStream(webcam,0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if(opModeIsActive()) {
            int tagID = 0;
            while (tagID != 1) {
                ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
                for (org.openftc.apriltag.AprilTagDetection detection : detections) {
                    tagID = detection.id;
                    if (tagID == 1) {
                        telemetry.addData("TAG FOUND: ", tagID);
                        break;
                    }
                    if (tagID != 1) {
                        telemetry.addData("Tag not Found.", "Nope");
                    }
                    telemetry.update();
                }
            }
        }

    }
}
