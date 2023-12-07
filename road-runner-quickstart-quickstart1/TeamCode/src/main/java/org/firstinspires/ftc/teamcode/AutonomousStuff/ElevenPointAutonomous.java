package org.firstinspires.ftc.teamcode.AutonomousStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "elevenPoints")
public class ElevenPointAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    @Override
    public void runOpMode() throws InterruptedException {

        FL  = hardwareMap.get(DcMotor.class, "FL");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
//        servo = hardwareMap.get(Servo.class, "servo");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()){
            strafe(0.5,200);
            brake(500);
           forward(0.5,1500);
           brake(500);
        }
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
