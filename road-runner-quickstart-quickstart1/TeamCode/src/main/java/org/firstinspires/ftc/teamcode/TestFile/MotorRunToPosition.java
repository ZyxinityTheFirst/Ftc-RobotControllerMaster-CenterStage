package org.firstinspires.ftc.teamcode.TestFile;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants.MotorConstants;

@TeleOp(name = "BrendanTrisPersonalMotorTester")
public class MotorRunToPosition extends LinearOpMode {

    private DcMotor leftMotor, rightMotor = null;
    private int runToAPosition = MotorConstants.postition;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (opModeIsActive()){

            runToAPosition = MotorConstants.postition;
            int rightRunToAPosition = MotorConstants.postition;

            leftMotor.setTargetPosition(runToAPosition);
            rightMotor.setTargetPosition(-rightRunToAPosition);
            if (gamepad1.x){
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftMotor.setPower(0.1);
                rightMotor.setPower(-0.1);
            }


            telemetry.addData("Position: ", runToAPosition);
            telemetry.addData("Position 1: Position 2:", "", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            telemetry.update();

            FtcDashboard.getInstance().updateConfig();
        }
    }
}
