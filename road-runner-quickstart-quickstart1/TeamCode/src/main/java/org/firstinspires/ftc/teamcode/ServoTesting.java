package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTestingPosition")
public class ServoTesting extends LinearOpMode {

    private double RightOpen;
    private double RightClose;
    private double LeftOpen;
    private double LeftClose;
    private double servoPos2;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() throws InterruptedException {

        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double currentServoPos = ServoConstants.closeServoPosRight;
            double currentServoPos1 = ServoConstants.openServoPosRight;
            double currentServoPos3 = ServoConstants.openServoPosLeft;
            double currentServoPos4 = ServoConstants.closeServoPosLeft;

            RightClose = currentServoPos;
            RightOpen = currentServoPos1;
            LeftClose = currentServoPos4;
            LeftOpen = currentServoPos3;

            if(gamepad1.a){
                rightServo.setPosition(RightClose);
            }
            else if(gamepad1.b){
                rightServo.setPosition(RightOpen);
            }
            else if(gamepad1.x){
                leftServo.setPosition(LeftOpen);
            }
            else if(gamepad1.y){
                leftServo.setPosition(LeftClose);
            }
            else if(gamepad1.dpad_down){
                rightServo.setPosition(RightOpen);
                leftServo.setPosition(LeftOpen);
            }
            else if(gamepad1.dpad_up){
                leftServo.setPosition(LeftClose);
                rightServo.setPosition(RightClose);
            }

            telemetry.addData("ServoDropperPos: ", RightClose);
            telemetry.addData("ServoDropperPos2: ", LeftOpen);
            telemetry.addData("ServoDropperPos3: ", RightClose);
            telemetry.addData("ServoDropperPos4: ", LeftClose);
            telemetry.addData("CurrentPos for rightServo: ", rightServo.getPosition());
            telemetry.addData("CurrentPos for leftServo: ", leftServo.getPosition());
            telemetry.update();

            FtcDashboard.getInstance().updateConfig();
        }
    }
}
