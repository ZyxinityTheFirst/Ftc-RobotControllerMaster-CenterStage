package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTesting")
public class LiftMotorTest extends LinearOpMode {

    private DcMotor leftMotor, rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class,  "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            double liftPower = gamepad1.left_trigger +- gamepad1.right_trigger;

            if (liftPower != 0) {
                leftMotor.setPower(liftPower);
                rightMotor.setPower(-liftPower);
            }
            else if(liftPower == 0){
                leftMotor.setPower(0.01);
                rightMotor.setPower(-0.01);
            }
        }
    }
}
