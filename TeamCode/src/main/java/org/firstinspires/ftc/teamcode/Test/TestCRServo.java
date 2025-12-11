package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CRServo Test", group = "Test")
public class TestCRServo extends OpMode {

    private CRServo CRServo;

    @Override
    public void init() {
        // Initialize the CRServo from the hardware map
        CRServo = hardwareMap.get(CRServo.class, "testCRServo");

        // Telemetry instructions

        telemetry.addLine("Use the Gamepad to control the servo:");
        telemetry.addLine("Right Trigger > 0.1 → Spin Forward");
        telemetry.addLine("Left Trigger > 0.1 → Spin Reverse");
        telemetry.addLine("Both released → Stop");
        telemetry.addLine("Press START to begin...");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Use the Gamepad to control the servo:");
        telemetry.addLine("Right Trigger > 0.1 → Spin Forward");
        telemetry.addLine("Left Trigger > 0.1 → Spin Reverse");
        telemetry.addLine("Both released → Stop");

        double power = 0.0;

        // Read input from gamepad
        if (gamepad1.right_trigger > 0.1) {
            power = gamepad1.right_trigger; // Forward
        } else if (gamepad1.left_trigger > 0.1) {
            power = -gamepad1.left_trigger; // Reverse
        } else {
            power = 0.0; // Stop
        }

        // Apply power to the servo
        CRServo.setPower(power);

        // Telemetry feedback
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("CRServo Power", power);
        telemetry.update();
    }
}