package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RoadRunnerTestFr")
public class RoadRunnerTest extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ColourConfig.QuadrantPipelineDetermination12 pipeline;
    private ColourConfig.QuadrantPipelineDetermination12.Quadrant12 snapshotAnalysis = ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE; // default


    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new ColourConfig.QuadrantPipelineDetermination12();
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d pose2d = new Pose2d(-42,-60, Math.toRadians(90));
        Pose2d redClose = new Pose2d(12.5, -60, Math.toRadians(90));
        Pose2d blueFar = new Pose2d(-42,60, Math.toRadians(-90));
        Pose2d blueClose = new Pose2d(12.5, 60, Math.toRadians(-90));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        drive.setPoseEstimate(pose2d);


//        Moves to parkingSpot for Far
     Trajectory redFarPark = drive.trajectoryBuilder(pose2d)
                .lineToConstantHeading(new Vector2d(48,-12.5))
                .build();





//        Red Close Quadrant 3 detection
        Trajectory redCloseRight = drive.trajectoryBuilder(redClose)
                .lineToConstantHeading(new Vector2d(-25,-40))
                .build();

        pose2d = redCloseRight.end();


//        Red Close Park
        Trajectory redClosePark = drive.trajectoryBuilder(pose2d)
                .lineToConstantHeading(new Vector2d(45,-60))
                .build();
        pose2d = redClosePark.end();


//        Part of center and leftFar for blue and red
        Trajectory zeroZero = drive.trajectoryBuilder(pose2d)
                .splineToLinearHeading(new Pose2d(0,0), Math.toRadians(-90))
                .build();

        pose2d = zeroZero.end();

        Trajectory blueFarPark = drive.trajectoryBuilder(pose2d)
                .lineToConstantHeading(new Vector2d(48,12.5))
                .build();

        Trajectory blueCloseRight = drive.trajectoryBuilder(blueClose)
                .lineToConstantHeading(new Vector2d(28,28))
                .build();
        Trajectory blueClosePark = drive.trajectoryBuilder(pose2d)
                .lineToConstantHeading(new Vector2d(64.5,52))
                        .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .splineToLinearHeading(new Pose2d(20,20), Math.toRadians(90))
                        .build();

        waitForStart();

        while(!opModeIsActive() && opModeInInit()){
            while (!isStarted() && !isStopRequested()) {

                telemetry.addData("Center X values: ", pipeline.getCenterX());
                telemetry.addData("RealTime Quadrant Analysis: ", pipeline.getQuadrant12());
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(500);
            }

        }
        snapshotAnalysis = pipeline.getQuadrant12();

        telemetry.addLine("Started.");
        telemetry.update();


        if(snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE){
            Pose2d newPose2d2 = new Pose2d(-42,-60, Math.toRadians(90));
            drive.setPoseEstimate(newPose2d2);

            //        Universal Center pathway for all
            Trajectory universalCenter = drive.trajectoryBuilder(newPose2d2)
                    .forward(40)
                    .build();
            newPose2d2 = universalCenter.end();

            Trajectory zeroZeroCenter = drive.trajectoryBuilder(newPose2d2)
                    .splineToLinearHeading(new Pose2d(0,0), Math.toRadians(-90))
                    .build();

            newPose2d2 = zeroZeroCenter.end();

            Trajectory redFarParkCenter = drive.trajectoryBuilder(newPose2d2)
                    .lineToConstantHeading(new Vector2d(48,-12.5))
                    .build();


        }

        if (opModeIsActive()){

            if(snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.TWO){
                Pose2d newPose2d = new Pose2d(-42,-60, Math.toRadians(90));

                drive.setPoseEstimate(newPose2d);

                //        Universal Center pathway for all
                Trajectory universalCenter = drive.trajectoryBuilder(newPose2d)
                        .forward(40)
                        .build();
                newPose2d = universalCenter.end();

                Trajectory zeroZeroCenter = drive.trajectoryBuilder(newPose2d)
                        .splineToLinearHeading(new Pose2d(0,0), Math.toRadians(-90))
                        .build();

                newPose2d = zeroZeroCenter.end();

                Trajectory redFarParkCenter = drive.trajectoryBuilder(newPose2d)
                        .lineToConstantHeading(new Vector2d(48,-12.5))
                        .build();

                drive.followTrajectory(universalCenter);
                drive.followTrajectory(zeroZero);
                drive.followTrajectory(redFarPark);
            }
            else if(snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE){
                Trajectory rightRedFar = drive.trajectoryBuilder(pose2d)
                        .lineToConstantHeading(new Vector2d(-50,-25))
                        .build();
                pose2d = rightRedFar.end();

                Trajectory zeroZeroRight = drive.trajectoryBuilder(pose2d)
                        .splineToLinearHeading(new Pose2d(0,0), Math.toRadians(-90))
                        .build();

                pose2d = zeroZeroRight.end();

                Trajectory redFarParkRight = drive.trajectoryBuilder(pose2d)
                        .lineToConstantHeading(new Vector2d(48,-12.5))
                        .build();



                drive.followTrajectory(rightRedFar);
                drive.followTrajectory(zeroZeroRight);
                drive.followTrajectory(redFarParkRight);
            }
            else{
                drive.setPoseEstimate(pose2d);

                //        Universal Center pathway for all
                Trajectory universalCenter = drive.trajectoryBuilder(pose2d)
                        .forward(40)
                        .build();
                pose2d = universalCenter.end();

                Trajectory zeroZeroCenter = drive.trajectoryBuilder(pose2d)
                        .splineToLinearHeading(new Pose2d(0,0), Math.toRadians(-90))
                        .build();

                pose2d = zeroZeroCenter.end();

                Trajectory redFarParkCenter = drive.trajectoryBuilder(pose2d)
                        .lineToConstantHeading(new Vector2d(48,-12.5))
                        .build();

                drive.followTrajectory(universalCenter);
                drive.followTrajectory(zeroZero);
                drive.followTrajectory(redFarPark);
            }

        }

    }

}
