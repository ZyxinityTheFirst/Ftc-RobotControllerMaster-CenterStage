package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BruhWhyBruh")
public class DriveCodeV2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor liftMotor = null;
    private DcMotor liftMotorMain = null;
    double mainLiftPower;
    double liftPower;
    boolean lockedIn = false;
    double speedLimiter = 1.0;
    private Servo planeServo;
    double servoPos = 0.1;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FL  = hardwareMap.get(DcMotor.class, "FL");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotorMain = hardwareMap.get(DcMotor.class, "liftMotorMain");
        planeServo = hardwareMap.get(Servo.class, "planeServo");

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);


        //Test with 15259 for encoder use.
        //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FL.setTargetPosition(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.dpad_down){
                speedLimiter = 2;
            }
            else if(gamepad1.dpad_up){
                speedLimiter = 1;
            }
            else if(gamepad1.dpad_right){
                speedLimiter = 1.65;
            }

            liftPower = gamepad1.left_trigger +- gamepad1.right_trigger;
            liftMotor.setPower(liftPower);

            if((gamepad1.left_trigger +- gamepad1.right_trigger == 0) && lockedIn){
                //Configure power level to keep robot in the air
                liftMotor.setPower(0.2);
            }
            else if((gamepad1.left_trigger +- gamepad1.right_trigger == 0)){
                liftMotor.setPower(0.069);
            }


            mainLiftPower = gamepad2.left_trigger +- gamepad2.right_trigger;
            liftMotorMain.setPower(mainLiftPower);

            if(mainLiftPower == 0){
                //Yet to configure
                liftMotorMain.setPower(0.2);
            }


//            if(gamepad1.a){
//                servo.setPosition(0);
//            }
//            else if (gamepad1.y){
//                servo.setPosition(0.3);
//            }
//            else if (gamepad1.b){
//                servo.setPosition(-1.5);
//            }

            if(gamepad1.x){
                planeServo.setPosition(servoPos);
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw     = -gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            leftFrontPower  /= speedLimiter;
            rightFrontPower /= speedLimiter;
            leftBackPower   /= speedLimiter;
            rightBackPower  /= speedLimiter;

            // Send calculated power to wheels
            FL.setPower(leftFrontPower);
            FR.setPower(rightFrontPower);
            BL.setPower(leftBackPower);
            BR.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("LiftMotor, MainLiftMotor", "%4.2f, 4.2f", liftMotor.getPower(), liftMotorMain.getPower());
            telemetry.addData("Current speedLimiter: ", speedLimiter);
            telemetry.addData("Servo position: ", planeServo.getPosition());
            telemetry.update();
        }
    }
}