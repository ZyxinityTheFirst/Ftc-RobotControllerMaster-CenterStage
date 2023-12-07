package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous (name = "RoadRunnerTestTrajectory")
public class RoadRunnerTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d pose2d = new Pose2d(-72,27, Math.toRadians(90));

        drive.setPoseEstimate(pose2d);

        Trajectory traj = drive.trajectoryBuilder(pose2d)
                .forward(36)
                .build();
        pose2d = drive.getPoseEstimate();
        Trajectory traj1 = drive.trajectoryBuilder(pose2d)
                .splineToLinearHeading(new Pose2d(45.0,36), Math.toRadians(-90))
                .build();

        pose2d = drive.getPoseEstimate();
        Trajectory traj2 = drive.trajectoryBuilder(pose2d)
                .lineToConstantHeading(new Vector2d(-45,54))
                .build();
        pose2d = drive.getPoseEstimate();

        Trajectory traj4 = drive.trajectoryBuilder(pose2d)
                .splineTo(new Vector2d(0,0),Math.toRadians(90))
                        .build();
        Trajectory traj5 = drive.trajectoryBuilder(pose2d)
                        .build();


        drive.followTrajectory(traj);
    }
}
