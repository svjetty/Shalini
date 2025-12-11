package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Bench", group = "Test")
public class TestBench extends OpMode {

    // Declare hardware
    private DcMotor testMotor;
    private Servo testServo;
    private CRServo testCRServo;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private TouchSensor touchSensor;

    private double servoPosition = 0.0;
    private double CRServoPower = 0.0;

    @Override
    public void init() {
        // Map hardware from config
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testServo = hardwareMap.get(Servo.class, "testServo");
        testCRServo = hardwareMap.get(CRServo.class, "testCRServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        testMotor.setPower(0);
        testCRServo.setPower(0);

        telemetry.addLine("== Initialization Complete ==");
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick Y: DC Motor");
        telemetry.addLine("Right Stick Y: CRServo");
        telemetry.addLine("Button A: Servo -> 0.0");
        telemetry.addLine("Button B: Servo -> 0.5");
        telemetry.addLine("Button X: Servo -> 1.0");
        telemetry.addLine("Live sensor data will show during loop.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Motor Control ===
        double motorPower = -gamepad1.left_stick_y;
        testMotor.setPower(motorPower);

        // === CRServo Control ===
        CRServoPower = -gamepad1.right_stick_y;
        testCRServo.setPower(CRServoPower);

        // === Standard Servo Control ===
        if (gamepad1.a) {
            servoPosition = 0.0;
        } else if (gamepad1.b) {
            servoPosition = 0.5;
        } else if (gamepad1.x) {
            servoPosition = 1.0;
        }
        testServo.setPosition(servoPosition);

        // === Telemetry ===
        telemetry.addLine("=== Live Data ===");
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.addData("CRServo Power", "%.2f", CRServoPower);
        telemetry.addData("Servo Position", "%.2f", servoPosition);

        telemetry.addLine("Color Sensor:");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Alpha", colorSensor.alpha());

        telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("Touch Sensor Pressed", touchSensor.isPressed() ? "YES" : "NO");

        telemetry.update();
    }
}
