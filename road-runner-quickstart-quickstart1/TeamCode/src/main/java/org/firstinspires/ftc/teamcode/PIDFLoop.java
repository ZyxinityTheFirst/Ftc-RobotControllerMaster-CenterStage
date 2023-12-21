package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.nio.file.attribute.FileTime;

@Config
@TeleOp(name = "PIDFLoop")
public class PIDFLoop extends OpMode {

    public PIDController controller;
    public static double p = 0, i = 0, d= 0;
    public static double f = 0;
    public static int target = 0;
    private final double tick_in_degree = 1680.0 / 360.0;
    private DcMotorEx leftMotor, rightMotor;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");
    }
    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = leftMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);

        double ff = Math.cos(Math.toRadians(target / tick_in_degree)) * f;

        double power = pid * ff;

        leftMotor.setPower(power);
        rightMotor.setPower(-power);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.addData("current left/right power ","%4.2f, %4.2f", leftMotor.getPower(), rightMotor.getPower());
        telemetry.update();
    }
}
