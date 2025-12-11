package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Sample Claw Test", group="TeleOp")
public class SampleClawTest extends OpMode {

    private Servo gripper;
    private Servo rotate;

    private boolean gripperClosed = false;
    private boolean aPressedLast = false;

    private double rotatePosition = 0.5;  // Start at middle position
    private final double DEADZONE = 0.05;  // Joystick dead zone

    @Override
    public void init() {
        gripper = hardwareMap.get(Servo.class, "gripper");
        rotate = hardwareMap.get(Servo.class, "rotate");

        gripper.setPosition(0.5);            // Start open
        rotate.setPosition(rotatePosition);  // Start centered
    }

    @Override
    public void loop() {
        // --- Gripper toggle using A button ---
        boolean aPressed = gamepad1.a;
        if (aPressed && !aPressedLast) {
            gripperClosed = !gripperClosed;
            gripper.setPosition(gripperClosed ? 1.0 : 0.55);
        }
        aPressedLast = aPressed;

        // --- Rotate servo control using left stick y ---

        double joystick = -gamepad1.left_stick_y;

        if (Math.abs(joystick) > DEADZONE) {
            rotatePosition += joystick * 0.01;  // Adjust sensitivity
            rotatePosition = Math.max(0.0, Math.min(1.0, rotatePosition));  // Clamp
            rotate.setPosition(rotatePosition);
        }

        telemetry.addData("Gripper", gripperClosed ? "Closed" : "Open");
        telemetry.addData("Rotate Pos", rotatePosition);
    }
}
