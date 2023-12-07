package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColourConfig;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name = "AutonomousForComp")
public class AutoComp extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ColourConfig.QuadrantPipelineDetermination12 pipeline;
    private DcMotor FL, FR, BR, BL = null;
    private Servo servoDropper = null;
    private final ElapsedTime runtime = new ElapsedTime();
    private ColourConfig.QuadrantPipelineDetermination12.Quadrant12 snapshotAnalysis = ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE; // default


    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        servoDropper = hardwareMap.get(Servo.class, "servoDropper");

        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

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

        while (!isStarted() && !isStopRequested()) {

            telemetry.addData("Center X values: ", pipeline.getCenterX());
            telemetry.addData("RealTime Quadrant Analysis: ", pipeline.getQuadrant12());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(500);
        }

        snapshotAnalysis = pipeline.getQuadrant12();

        if (opModeIsActive()) {

            if (snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.ONE) {
                //Left path
                forward(0.5, 600);
                brake(500);
                rotate(0.55, 450);
                brake(500);
                forward(0.5, 200);
                brake(500);
                forward(0.5, 1200);
                brake(500);
                strafe(-0.5,700);
                brake(500);
            } else if (snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.TWO) {
                //Center Path
                forward(0.5, 850 );
                brake(800);
                forward(-0.5, 300);
                brake(500);
                rotate(0.5, 600);
                brake(500);
                forward(0.5, 1500);
                brake(500);
                strafe(-0.5, 650);
                brake(500);
            } else if(snapshotAnalysis == ColourConfig.QuadrantPipelineDetermination12.Quadrant12.THREE) {
                //Right path
                forward(0.5, 600);
                brake(500);
                rotate(-0.55, 550);
                brake(500);
                forward(0.5, 200);
                brake(500);
                forward(-0.5, 2500);
                brake(500);
                strafe(0.5,700);
                brake(500);
            }
            else {
                //Default path yet to be decided.
            }
        }
        FtcDashboard.getInstance().stopCameraStream();

    }

    public void forward(double power, int time){
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(power);
        BL.setPower(power);
        sleep(time);
    }

    public void strafe(double power, int time){
        //goes left by default
        FR.setPower(power);
        BR.setPower(-power);
        FL.setPower(-power);
        BL.setPower(power);
        sleep(time);
    }
    public void diagonalLeft(double power, int time){
        //goes diagonally left by default -power results in the robot moving diagonalRight backwards
        FR.setPower(power);
        BL.setPower(power);
        sleep(time);
    }
    public void diagonalRight(double power, int time){
        //goes diagonally right by default -powers result in the robot moving diagonalLeft backwards
        BR.setPower(power);
        FL.setPower(power);
        sleep(time);
    }
    public void rotate(double power, int time){
        //Rotates Left by default
        FR.setPower(power);
        BR.setPower(power);
        FL.setPower(-power);
        BL.setPower(-power);
        sleep(time);
    }

    public void brake(int time){
        //goes left by default
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        sleep(time);
    }



}