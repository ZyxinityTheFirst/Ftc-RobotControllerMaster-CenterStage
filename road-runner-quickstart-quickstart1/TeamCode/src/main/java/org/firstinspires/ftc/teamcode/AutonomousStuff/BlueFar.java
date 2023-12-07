package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "BlueFarSplineTest")
public class BlueFar extends LinearOpMode {

        private Servo servoDropper;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d blueFarSpline1 = new Pose2d(-37,61, Math.toRadians(-90));

        servoDropper = hardwareMap.get(Servo.class,"servoDropper");

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
//        TrajectorySequence blueFarLeft = drive.trajectorySequenceBuilder(blueFarSpline1)
//                .splineTo(new Vector2d(-18, 31), Math.toRadians(0))
//                .addTemporalMarker(() ->{
//                    dropServo(servoDropper);
//                })
//                .waitSeconds(1.0)
//                .forward(6)
//                .lineToConstantHeading(new Vector2d(-11,15))
//                .splineToConstantHeading(new Vector2d(46,20), Math.toRadians(90))
//                .strafeLeft(20)
//                .waitSeconds(1.0)
//                .strafeRight(20)
//                .lineToLinearHeading(new Pose2d(-58,-18,Math.toRadians(180)))
//                .waitSeconds(0.5)
//                .lineToLinearHeading(new Pose2d(50,22, Math.toRadians(0)))
//                .strafeLeft(14)
//                .waitSeconds(1.0)
//                .strafeRight(18)
//                .build();

        waitForStart();

        servoDropper.setPosition(0.8);

        if (opModeIsActive()){
//            drive.followTrajectorySequence(blueFarSpline);
//            drive.followTrajectorySequence(blueFarCenter);
//            drive.followTrajectorySequence(blueFarLeft);
        }

    }

    public void dropServo(Servo servoDropper){
        this.servoDropper = servoDropper;
        servoDropper.setPosition(0.35);
    }
    public void closeServo(Servo servoDropper){
        this.servoDropper = servoDropper;
        servoDropper.setPosition(0.8);
    }
}
