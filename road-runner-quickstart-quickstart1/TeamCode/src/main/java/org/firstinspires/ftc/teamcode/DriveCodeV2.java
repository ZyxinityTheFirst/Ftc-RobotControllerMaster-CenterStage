package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.ServoConstants;

@TeleOp(name = "BruhWhyTheSecond")
public class DriveCodeV2 extends LinearOpMode {

    //Declare our PIDController for later use.
    private PIDController controller;

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor leftMotor, rightMotor = null;
    private double closeLeft, closeRight, openRight, openLeft;
    double liftPower = 0;
    double speedLimiter = 1.0;
    private Servo planeServo, leftServo, rightServo;
    private boolean isOpen = false;
    private boolean wasPressed = false;
    private double p = PIDFLoop.p, i = PIDFLoop.i,d = PIDFLoop.d;
    private double f = PIDFLoop.f;
    private double target = 0;
    private final double tick_in_degree = 1680.0 / 360.0;

    @Override
    public void runOpMode() {

        int armPos = leftMotor.getCurrentPosition();

        openLeft = ServoConstants.openServoPosLeft;
        openRight = ServoConstants.openServoPosRight;
        closeLeft = ServoConstants.closeServoPosLeft;
        closeRight = ServoConstants.closeServoPosRight;

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class,  "rightMotor");

        planeServo = hardwareMap.get(Servo.class, "planeServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.dpad_down) {
                speedLimiter = 2;
            } else if (gamepad1.dpad_up) {
                speedLimiter = 1;
            } else if (gamepad1.dpad_right) {
                speedLimiter = 1.65;
            }

            boolean isPressed = gamepad1.left_bumper;

            if(isPressed && !wasPressed){
                if(isOpen){
                    leftServo.setPosition(closeLeft);
                    rightServo.setPosition(closeRight);
                } else {
                    leftServo.setPosition(openLeft);
                    rightServo.setPosition(openRight);
                }
                isOpen = !isOpen; // Toggle the state
            }

            wasPressed = isPressed; // Update the previous state of the button

            liftPower = -gamepad2.right_stick_y;

            if (liftPower < 0) {
                liftPower /= 2;
                leftMotor.setPower(liftPower);
                rightMotor.setPower(-liftPower);
            }
            else if(liftPower > 0){
                liftPower /= 4;
                leftMotor.setPower(liftPower);
                rightMotor.setPower(-liftPower);
            }
            else if(liftPower == 0){
                controller.setPID(p,i,d);
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / tick_in_degree)) * f;
                double power = pid * ff;

                leftMotor.setPower(power);
                rightMotor.setPower(power);
            }

            if (gamepad1.x) {
                planeServo.setPosition(0.8);   //Parth Patel was here
            }


            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontPower /= speedLimiter;
            rightFrontPower /= speedLimiter;
            leftBackPower /= speedLimiter;
            rightBackPower /= speedLimiter;

            // Send calculated power to wheels
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Current speedLimiter: ", speedLimiter);
            telemetry.addData("Servo position: ", planeServo.getPosition());
            telemetry.addData("liftPower: ", liftPower);
            telemetry.update();
        }
    }
}