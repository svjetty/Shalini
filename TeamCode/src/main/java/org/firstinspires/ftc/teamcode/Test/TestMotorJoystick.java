package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test - Joystick", group = "Test")
public class TestMotorJoystick extends OpMode {

    private DcMotor testMotor;

    @Override
    public void init() {
        // Initialize motor from configuration
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Motor Test Initialized");
        telemetry.addLine("-> Use LEFT joystick (Y-axis) to control motor power.");
        telemetry.addLine("-> Press START to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addLine("-> Use LEFT joystick (Y-axis) to control motor power.");

        // Get joystick Y input and invert it (up is negative)
        double power = -gamepad1.left_stick_y;

        // Apply power to the motor
        testMotor.setPower(power);

        // Display telemetry data
        telemetry.addLine("Motor Test Running");
        telemetry.addData("Joystick Y", gamepad1.left_stick_y);
        telemetry.addData("Motor Power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop motor on OpMode stop
        testMotor.setPower(0);
    }
}
