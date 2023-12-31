//package org.firstinspires.ftc.teamcode.AutonomousStuff;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.AutonomousStuff.ColourConfig;
//import org.firstinspires.ftc.teamcode.Constants.MotorConstants;
//import org.firstinspires.ftc.teamcode.Constants.ServoConstants;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//@Autonomous(name = "redCloseSpline")
//public class RedClose extends LinearOpMode {
//    private OpenCvWebcam webcam;
//    private ColourConfig.QuadrantPipelineDetermination12 pipeline;
//    private ColourConfig.QuadrantPipelineDetermination12.Quadrant12 snapshotAnalysis = ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE; // default
//    private Servo servoDropper;
//    private Servo leftServo, rightServo;
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftMotor, rightMotor = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d redClose = new Pose2d(14, -61, Math.toRadians(90));
//
//        servoDropper = hardwareMap.get(Servo.class, "servoDropper");
//        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
//
//
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
//        pipeline = new ColourConfig.QuadrantPipelineDetermination12();
//        webcam.setPipeline(pipeline);
//
//        drive.setPoseEstimate(redClose);
//
//
//        //HARDEST PATHWAY DONE
//        TrajectorySequence redCloseLeft = drive.trajectorySequenceBuilder(redClose)
//                .addTemporalMarker(0, () -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//                })
//                .splineToLinearHeading(new Pose2d(7.5, -35), Math.toRadians(180))
//                .addTemporalMarker(() -> {
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//
//                //*, Math.toRadians(-90)*/
//                .lineToConstantHeading(new Vector2d(45, -23))
//                .addTemporalMarker(2.5, () -> {
//
//                    leftMotor.setTargetPosition(MotorConstants.backBoard);
//                    rightMotor.setTargetPosition(-MotorConstants.backBoard);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//
//                })
//                .waitSeconds(2.0)
//                .addTemporalMarker(() -> {
//                    dropServoV2(leftServo, rightServo, ServoConstants.openServoPosLeft, ServoConstants.openServoPosRight);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.2);
//                    rightMotor.setPower(-0.2);
//                })
//                .back(5)
////                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(-90)))
////                //Stack
//////                                        .lineToLinearHeading(new Pose2d(-60,-13, Math.toRadians(180)))
//////                                        .waitSeconds(0.5)
//////                                        .lineToLinearHeading(new Pose2d(50,36,Math.toRadians(0)))
//////                                        .waitSeconds(0.5)
////
////                //UNCOMMENT FOR 4 PIXELS
////                .lineToLinearHeading(new Pose2d(-61,-10, Math.toRadians(180)))
////                .waitSeconds(0.5)
////                .lineToConstantHeading(new Vector2d(0,0))
//////                .lineToLinearHeading(new Pose2d(48,40,Math.t  oRadians(0)))
////                .splineToSplineHeading(new Pose2d(46,35, Math.toRadians(0)), Math.toRadians(90))
////                .waitSeconds(1.0)
//                .strafeRight(35)
//                .forward(15)
//
//                //Park :down_arrow:
//                //Comment out if going for 4 pixels >:)
////                                        .strafeRight(15)
////                                        .forward(5)
//                .build();
//
//
//        //BLUE RIGHT
//        TrajectorySequence redCloseRight = drive.trajectorySequenceBuilder(redClose)
//                .addTemporalMarker(0, () -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//                })
//                .splineTo(new Vector2d(29, -29), Math.toRadians(0))
////              .lineToConstantHeading(new Vector2d(23,25))
//                .addTemporalMarker(() -> {
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//
//                //*, Math.toRadians(-90)*/
//                .lineToConstantHeading(new Vector2d(46.5, -41))
////                                        .lineToLinearHeading(new Pose2d(48,40,Math.toRadians(0)))
//                .addTemporalMarker(2.5, () -> {
//
//                    leftMotor.setTargetPosition(MotorConstants.backBoard);
//                    rightMotor.setTargetPosition(-MotorConstants.backBoard);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//
//                })
//                .waitSeconds(2.0)
//                .addTemporalMarker(() -> {
//                    dropServoV2(leftServo, rightServo, ServoConstants.openServoPosLeft, ServoConstants.openServoPosRight);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.2);
//                    rightMotor.setPower(-0.2);
//                })
//                .back(5)
//
////                //Stack
////                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(180))
////                .lineToLinearHeading(new Pose2d(-61,3, Math.toRadians(180)))
//////                                        .lineToLinearHeading(new Pose2d(-58,-11, Math.toRadians(180)))
////                .waitSeconds(1.0)
////                .lineToLinearHeading(new Pose2d(46,3,Math.toRadians(0)))
////                .lineToLinearHeading(new Pose2d(46,38,Math.toRadians(0)))
////                .waitSeconds(1.0)
////                .lineToConstantHeading(new Vector2d(46,58))
//                //UNCOMMENT FOR 4 PIXELS
////                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
////                                        .waitSeconds(0.5)
////                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
//
//
//                //Park :down_arrow:
//                //Comment out if going for 4 pixels >:)
//                .strafeRight(25)
//                .forward(25)
//                .build();
//
//
//        //CENTER
//        TrajectorySequence redCloseCenter = drive.trajectorySequenceBuilder(redClose)
//                .addTemporalMarker(0, () -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//                })
//                .forward(41)
//                .addTemporalMarker(() -> {
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(1.0)
//                .back(20.5)
//                //*, Math.toRadians(-90)*/s
//                .lineToLinearHeading(new Pose2d(49, -29.5, Math.toRadians(0)))
//                .addTemporalMarker(() -> {
//
//                    leftMotor.setTargetPosition(MotorConstants.backBoard);
//                    rightMotor.setTargetPosition(-MotorConstants.backBoard);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.1);
//                    rightMotor.setPower(-0.1);
//
//                })
//                .waitSeconds(4.0)
//                .addTemporalMarker(() -> {
//                    dropServoV2(leftServo, rightServo, ServoConstants.openServoPosLeft, ServoConstants.openServoPosRight);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    leftMotor.setTargetPosition(MotorConstants.rest);
//                    rightMotor.setTargetPosition(-MotorConstants.rest);
//
//                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    leftMotor.setPower(0.2);
//                    rightMotor.setPower(-0.2);
//                })
//                .back(5)
//                //Stack
////                .lineToLinearHeading(new Pose2d(-58,-18, Math.toRadians(180)))
////                .waitSeconds(0.5)
////                .lineToLinearHeading(new Pose2d(46,20,Math.toRadians(0)))
////                .strafeLeft(12)
////                .waitSeconds(1.0)
//
//                //UNCOMMENT FOR 4 PIXELS
////                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
////                                        .waitSeconds(0.5)
////                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
//
//
//                //Park :down_arrow:
//                //Comment out if going for 4 pixels >:)
//                .strafeRight(30)
//                .forward(20)
//                .build();
//
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Init Error", errorCode);
//                telemetry.update();
//            }
//        });
//        FtcDashboard.getInstance().startCameraStream(webcam, 0);
//
//        dropServoV2(leftServo, rightServo, ServoConstants.closeServoPosLeft, ServoConstants.closeServoPosRight);
//
//        while (!isStarted() && !isStopRequested()) {
//
//            telemetry.addData("Center X values: ", pipeline.getCenterX());
//            telemetry.addData("RealTime Quadrant Analysis: ", pipeline.getQuadrant12());
//            telemetry.update();
//
//
//            snapshotAnalysis = pipeline.getQuadrant12();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(500);
//        }
//
//        FtcDashboard.getInstance().stopCameraStream();
//        waitForStart();
//
//        if (opModeIsActive()) {
//            if (snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE) {
//                drive.followTrajectorySequence(redCloseLeft);
//            } else if (snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.THREE) {
//                drive.followTrajectorySequence(redCloseRight);
//            } else if (snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.TWO) {
//                drive.followTrajectorySequence(redCloseCenter);
//            }
//        }
//    }
//
//    public void dropServo(Servo servo){
//        servo.setPosition(0.35);
//    }
//    public void dropServoV2(Servo servo, Servo servo1, double power, double power2){
//        servo.setPosition(power);
//        servo1.setPosition(power2);
//    }
//    public void closeServo(Servo servo){
//        servo.setPosition(0.8);
//    }
//
//}
