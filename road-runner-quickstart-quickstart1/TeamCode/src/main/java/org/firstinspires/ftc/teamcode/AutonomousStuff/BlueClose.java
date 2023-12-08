package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.MotorConstants;
import org.firstinspires.ftc.teamcode.Constants.ServoConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

@Autonomous(name = "BlueCloseSpline")
public class BlueClose extends LinearOpMode {
    private Servo servoDropper;
    private Servo leftServo, rightServo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueClose = new Pose2d(14, 61, Math.toRadians(-90));

        servoDropper = hardwareMap.get(Servo.class,"servoDropper");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPoseEstimate(blueClose);



        //BlueCloseCenter
//        TrajectorySequence blueCloseCenter = drive.trajectorySequenceBuilder(blueClose)
//                .forward(41)
//                .addTemporalMarker(() ->{
//                dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//                //*, Math.toRadians(-90)*/
//                .lineToLinearHeading(new Pose2d(46,25,Math.toRadians(0)))
//                .strafeLeft(10)
//                .waitSeconds(0.5)
//                .strafeRight(15)
//
//                //Stack
//                .lineToLinearHeading(new Pose2d(-58,-18, Math.toRadians(180)))
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(46,20,Math.toRadians(0)))
//                .strafeLeft(12)
//                .waitSeconds(1.0)
//
//                //UNCOMMENT FOR 4 PIXELS
////                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
////                                        .waitSeconds(0.5)
////                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
//
//
//                //Park :down_arrow:
//                //Comment out if going for 4 pixels >:)
//                .strafeLeft(30)
//                .build();


        //blueCloseLeft
//        TrajectorySequence blueCloseLeft = drive.trajectorySequenceBuilder(blueClose)
//                .splineTo(new Vector2d(29,27), Math.toRadians(0))
////              .lineToConstantHeading(new Vector2d(23,25))
//                .addTemporalMarker(() ->{
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//
//                //*, Math.toRadians(-90)*/
//                .lineToConstantHeading(new Vector2d(46,40))
////                                        .lineToLinearHeading(new Pose2d(48,40,Math.toRadians(0)))
//                .waitSeconds(0.5)
//                .strafeRight(10)
//
//                //Stack
//                .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(180))
//                .lineToLinearHeading(new Pose2d(-61,3, Math.toRadians(180)))
////                                        .lineToLinearHeading(new Pose2d(-58,-11, Math.toRadians(180)))
//                .waitSeconds(1.0)
//                .lineToLinearHeading(new Pose2d(46,3,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(46,38,Math.toRadians(0)))
//                .waitSeconds(1.0)
//                .lineToConstantHeading(new Vector2d(46,58))
//                //UNCOMMENT FOR 4 PIXELS
////                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
////                                        .waitSeconds(0.5)
////                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
//
//
//                //Park :down_arrow:
//                //Comment out if going for 4 pixels >:)
//                .build();

        TrajectorySequence blueCloseRight = drive.trajectorySequenceBuilder(blueClose)
                .addTemporalMarker(0, ()->{
                    leftMotor.setTargetPosition(MotorConstants.rest);
                    rightMotor.setTargetPosition(-MotorConstants.rest);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.1);
                    rightMotor.setPower(-0.1);
                })
                .splineToLinearHeading(new Pose2d(7.5,33), Math.toRadians(180))
                .addTemporalMarker(() ->{
                    dropServo(servoDropper);
                })
                .waitSeconds(0.5)
                //*, Math.toRadians(-90)*/
                .lineToConstantHeading(new Vector2d(48.5,26))
                .addTemporalMarker(2.5,() ->{

                    leftMotor.setTargetPosition(MotorConstants.backBoard);
                    rightMotor.setTargetPosition(-MotorConstants.backBoard);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.1);
                    rightMotor.setPower(-0.1);

                })
                .waitSeconds(1.0)
                .addTemporalMarker(() ->{
                    dropServoV2(leftServo,rightServo, ServoConstants.openServoPosLeft, ServoConstants.openServoPosRight);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    leftMotor.setTargetPosition(MotorConstants.rest);
                    rightMotor.setTargetPosition(-MotorConstants.rest);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.1);
                    rightMotor.setPower(-0.1);
                })
                .back(5)
//                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(-90)))
//                //Stack
////                                        .lineToLinearHeading(new Pose2d(-60,-13, Math.toRadians(180)))
////                                        .waitSeconds(0.5)
////                                        .lineToLinearHeading(new Pose2d(50,36,Math.toRadians(0)))
////                                        .waitSeconds(0.5)
//
//                //UNCOMMENT FOR 4 PIXELS
//                .lineToLinearHeading(new Pose2d(-61,-10, Math.toRadians(180)))
//                .waitSeconds(0.5)
//                .lineToConstantHeading(new Vector2d(0,0))
////                .lineToLinearHeading(new Pose2d(48,40,Math.t  oRadians(0)))
//                .splineToSplineHeading(new Pose2d(46,35, Math.toRadians(0)), Math.toRadians(90))
//                .waitSeconds(1.0)
                .strafeLeft(30)
                .forward(15)

                //Park :down_arrow:
                //Comment out if going for 4 pixels >:)
//                                        .strafeRight(15)
//                                        .forward(5)
                .build();


        if(!opModeIsActive() && opModeInInit()) {
            dropServoV2(leftServo, rightServo, ServoConstants.closeServoPosLeft, ServoConstants.closeServoPosRight);
        }

        waitForStart();


        if (opModeIsActive()){
//            drive.followTrajectorySequence(blueCloseCenter);
//            drive.followTrajectorySequence(blueCloseLeft);
            drive.followTrajectorySequence(blueCloseRight);
        }

    }
    public void dropServo(Servo servo){
        servo.setPosition(0.35);
    }
    public void dropServoV2(Servo servo, Servo servo1, double power, double power2){
        servo.setPosition(power);
        servo1.setPosition(power2);
    }
    public void closeServo(Servo servo){
        servo.setPosition(0.8);
    }
}
