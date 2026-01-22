package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ShooterVelocityWithServo")
public class ShooterVelocityWithServo extends OpMode {

    DcMotorEx left, right;
    Servo angleServo;

    boolean shooterOn = false;

    // Shooter RPM control
    double targetRPM = 3000;   // starting RPM
    double rpmStep = 100;      // how much to increase/decrease

    // goBILDA bare motor encoder
    final double TICKS_PER_REV = 28.0;

    // Button debouncing
    boolean lastA = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLB = false;
    boolean lastRB = false;

    // Servo angle
    double servoPos = 0.5;
    double servoStep = 0.02;

    @Override
    public void init() {

        left  = hardwareMap.get(DcMotorEx.class, "shooterL");
        right = hardwareMap.get(DcMotorEx.class, "shooterR");
        angleServo = hardwareMap.get(Servo.class, "angle");

        // Reverse left motor if needed
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Shooter Velocity Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // -------- Toggle shooter (A button) --------
        boolean a = gamepad1.a;
        if (a && !lastA) shooterOn = !shooterOn;
        lastA = a;

        // -------- RPM increase --------
        boolean up = gamepad1.dpad_up;
        if (up && !lastUp) {
            targetRPM += rpmStep;
        }
        lastUp = up;

        // -------- RPM decrease --------
        boolean down = gamepad1.dpad_down;
        if (down && !lastDown) {
            targetRPM -= rpmStep;
            if (targetRPM < 0) targetRPM = 0;
        }
        lastDown = down;

        // -------- Convert RPM → ticks/sec --------
        double targetTPS = (targetRPM * TICKS_PER_REV) / 60.0;

        // -------- Apply velocity --------
        if (shooterOn) {
            left.setVelocity(targetTPS);
            right.setVelocity(targetTPS);
        } else {
            left.setVelocity(0);
            right.setVelocity(0);
        }

        // -------- Servo control --------
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

        // -------- Telemetry --------
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Target RPM", (int) targetRPM);
        telemetry.addData("Servo Pos", "%.2f", servoPos);
        telemetry.update();
    }
}
