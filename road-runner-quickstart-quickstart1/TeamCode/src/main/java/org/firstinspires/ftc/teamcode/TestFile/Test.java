package org.firstinspires.ftc.teamcode.TestFile;

import static org.firstinspires.ftc.teamcode.PIDFLoop.d;
import static org.firstinspires.ftc.teamcode.PIDFLoop.f;
import static org.firstinspires.ftc.teamcode.PIDFLoop.i;
import static org.firstinspires.ftc.teamcode.PIDFLoop.p;
import static org.firstinspires.ftc.teamcode.PIDFLoop.target;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Yeah")
public class Test extends LinearOpMode {

    private PIDController controller;
    private DcMotorEx leftMotor, rightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        controller.setPID(p,i,d);
        int armPos = leftMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / (1680.0/ 360.0))) * f;

        double power = pid * ff;

        leftMotor.setPower(power);
        rightMotor.setPower(power);

    }
}
