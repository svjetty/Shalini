package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SimpleShooterWithServo")
public class SimpleShooterWithServo extends OpMode {

    DcMotorEx left, right;
    Servo angleServo;

    boolean shooterOn = false;
    double power = 0.5;
    double step = 0.01;

    double servoPos = 0.0;        // middle position
    double servoStep = 0.05;      // how much angle changes each press

    boolean lastA = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLB = false;
    boolean lastRB = false;

    @Override
    public void init() {
        left  = hardwareMap.get(DcMotorEx.class, "shooterL");
        right = hardwareMap.get(DcMotorEx.class, "shooterR");
        angleServo = hardwareMap.get(Servo.class, "angle");

        // Reverse left motor if needed
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Shooter + Servo Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // -------- Shooter Toggle: A --------
        boolean a = gamepad1.a;
        if (a && !lastA) shooterOn = !shooterOn;
        lastA = a;

        // -------- Power Up --------
        boolean up = gamepad1.dpad_up;
        if (up && !lastUp) {
            power += step;
            if (power > 1) power = 1;
        }
        lastUp = up;

        // -------- Power Down --------
        boolean down = gamepad1.dpad_down;
        if (down && !lastDown) {
            power -= step;
            if (power < 0) power = 0;
        }
        lastDown = down;

        // -------- Servo Angle Control --------
        boolean lb = gamepad1.left_bumper;
        if (lb && !lastLB) {
            servoPos -= servoStep;
            if (servoPos < 0) servoPos = 0;
        }
        lastLB = lb;

        boolean rb = gamepad1.right_bumper;
        if (rb && !lastRB) {
            servoPos += servoStep;
            if (servoPos > 1) servoPos = 1;
        }
        lastRB = rb;

        angleServo.setPosition(servoPos);

        // -------- Apply Shooter Power --------
        if (shooterOn) {
            left.setPower(power);
            right.setPower(power);
        } else {
            left.setPower(0);
            right.setPower(0);
        }

        // -------- Telemetry --------
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Shooter Power", "%.2f", power);
        telemetry.addData("Servo Position", "%.2f", servoPos);
        telemetry.update();
    }
}
