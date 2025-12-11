package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class TestServo extends OpMode {

    private Servo testServo;
    private double position = 0.5;

    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "testServo");
        testServo.setPosition(position); // Set initial position

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press A=0.0, B=0.5, X=1.0");
        telemetry.addData("Initial Servo Position", position);
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Press A=0.0, B=0.5, X=1.0");
        telemetry.addData("Initial Servo Position", position);

        if (gamepad1.a) {
            position = 0.0;
        } else if (gamepad1.b) {
            position = 0.5;
        } else if (gamepad1.x) {
            position = 1.0;
        }

        testServo.setPosition(position);
        telemetry.addData("Servo Position", position);
        telemetry.update();
    }
}
