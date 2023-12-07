package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "AutonomousOnTheAutonomous")
public class Autonomous extends LinearOpMode {

    private DcMotor BR, BL, FR, FL = null;
    private final ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);


       runtime.startTime();


       waitForStart();
       runtime.reset();

       strafe(-0.5, 500);
       brake();
       forward(.5, 500);
       brake();

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
        //goes left by default
        FR.setPower(power);
        BL.setPower(power);
        sleep(time);
    }
    public void diagonalRight(double power, int time){
        //goes left by default
        BR.setPower(power);
        FL.setPower(power);
        sleep(time);
    }
    public void brake(){
        //goes left by default
        FR.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        BL.setPower(0);
        sleep(500);
    }
}
