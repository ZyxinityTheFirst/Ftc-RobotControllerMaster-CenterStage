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

@Autonomous(name = "BlueFarSplineTest")
public class BlueFar extends LinearOpMode {
    private Servo servoDropper;
    private Servo leftServo, rightServo;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor, rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueFarSpline1 = new Pose2d(-37,61, Math.toRadians(-90));

        servoDropper = hardwareMap.get(Servo.class,"servoDropper");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        drive.setPoseEstimate(blueFarSpline1);

        //Right far spline
//        TrajectorySequence blueFarSpline = drive.trajectorySequenceBuilder(new Pose2d(-37,61, Math.toRadians(-90)))
//                .lineToConstantHeading(new Vector2d(-51,23))
//                .addTemporalMarker(() -> {
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//                //*, Math.toRadians(-90)*/
//                .splineToConstantHeading(new Vector2d(-5, 0), Math.toRadians(0))
//                .lineToLinearHeading(new Pose2d(48,25,Math.toRadians(0)))
//                .strafeLeft(7)
//                .waitSeconds(1.0)
//                .strafeRight(7)
//                //Stack
//                .lineToLinearHeading(new Pose2d(-55,-12, Math.toRadians(180)))
//                .waitSeconds(1.0)
//                .lineToLinearHeading(new Pose2d(48,25,Math.toRadians(0)))
//                .strafeLeft(12)
//                .waitSeconds(1.0)
//                //Park :down_arrow:
//                .strafeRight(15)
//                .build();


        //BlueFarCenterSpline
//        TrajectorySequence blueFarCenter = drive.trajectorySequenceBuilder(blueFarSpline1)
//                .forward(40)
//                .addTemporalMarker(() ->{
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(0.5)
//                .forward(8)
//                .splineTo(new Vector2d(-5,0), Math.toRadians(0))
//                .splineTo(new Vector2d(46,25), Math.toRadians(0))
//                .strafeLeft(7)
//                .waitSeconds(1.0)
//                .strafeRight(7)
//                .waitSeconds(1.0)
//                .lineToLinearHeading(new Pose2d(10,0, Math.toRadians(180)))
//                .splineTo(new Vector2d(-55,-15), Math.toRadians(180))
//                .waitSeconds(1.0)
//                .lineToConstantHeading(new Vector2d(0,0))
//                .splineToSplineHeading(new Pose2d(46,25, Math.toRadians(0)), Math.toRadians(90))
//                .strafeLeft(12)
//                .waitSeconds(1.0)
//                .strafeRight(22)
//                .build();


        //BlueFarLeft
        TrajectorySequence blueFarLeft = drive.trajectorySequenceBuilder(blueFarSpline1)
                .addTemporalMarker(0, ()->{
                    leftMotor.setTargetPosition(MotorConstants.rest);
                    rightMotor.setTargetPosition(-MotorConstants.rest);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.1);
                    rightMotor.setPower(-0.1);
                })
                .splineTo(new Vector2d(-20, 33), Math.toRadians(0))
                .addTemporalMarker(() ->{
                    dropServo(servoDropper);
                })
                .waitSeconds(1.0)
                .forward(10)
                .lineToConstantHeading(new Vector2d(-11,15))
                .splineToConstantHeading(new Vector2d(46,38), Math.toRadians(90))
//                .strafeLeft(20)
//                .waitSeconds(1.0)
//                .strafeRight(20)
//                .lineToLinearHeading(new Pose2d(-58,-18,Math.toRadians(180)))
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(45,26.5, Math.toRadians(0)))
                .addTemporalMarker(() ->{

                    leftMotor.setTargetPosition(MotorConstants.backBoard);
                    rightMotor.setTargetPosition(-MotorConstants.backBoard);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.15);
                    rightMotor.setPower(-0.15);

                })
                .waitSeconds(2.5)
                .addTemporalMarker(() ->{
                    dropServoV2(leftServo,rightServo, ServoConstants.openServoPosLeft, ServoConstants.openServoPosRight);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() ->{
                    leftMotor.setTargetPosition(MotorConstants.rest);
                    rightMotor.setTargetPosition(-MotorConstants.rest);

                    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    leftMotor.setPower(0.2);
                    rightMotor.setPower(-0.2);
                })
                .back(5)
                .strafeRight(10)
                .waitSeconds(1.0)
                .forward(10)
                .build();

        if(!opModeIsActive() && opModeInInit()) {
            dropServoV2(leftServo, rightServo, ServoConstants.closeServoPosLeft, ServoConstants.closeServoPosRight);
        }

        waitForStart();

        if (opModeIsActive()){
//            drive.followTrajectorySequence(blueFarSpline);
//            drive.followTrajectorySequence(blueFarCenter);
            drive.followTrajectorySequence(blueFarLeft);
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
