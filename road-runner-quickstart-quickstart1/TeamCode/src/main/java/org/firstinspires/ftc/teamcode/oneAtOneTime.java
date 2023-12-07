package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name = "FindThyMotors")
public class oneAtOneTime extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL, FR, BR, BL = null;

    @Override
    public void runOpMode() throws InterruptedException {


        FL  = hardwareMap.get(DcMotor.class, "FL");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            if (gamepad1.a){
                FL.setPower(0.5);
                sleep(100);
                FL.setPower(0);
            }
            else if (gamepad1.b){
                //FR == BR
                FR.setPower(0.5);
                sleep(100);
                FR.setPower(0);
            }
            else if (gamepad1.x){
                BR.setPower(0.5);
                sleep(100);
                BR.setPower(0);
            }
            else if (gamepad1.y){
                BL.setPower(0.5);
                sleep(100);
                BL.setPower(0);
            }

        }

    }
}
