package org.firstinspires.ftc.teamcode.Toggle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Button Toggle Test - Simple", group="Toggle")
public class ButtonToggleTestSimple extends OpMode {

    private DcMotor testMotor;
    private boolean motorOn = false;
    private boolean aPressedLast = false;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        telemetry.addLine("=== Motor Toggle Test ===");
        telemetry.addLine("Press 'A' on Gamepad1 to toggle the motor ON/OFF.");
        telemetry.addLine("Motor name in config: 'testMotor'");
        telemetry.addLine("---------------------------");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Read gamepad A button
        boolean aPressed = gamepad1.a;

        // Detect rising edge of button press (debounce)
        if (aPressed && !aPressedLast) {
            motorOn = !motorOn; // Toggle motor state
        }
        aPressedLast = aPressed;

        // Set motor power based on state
        testMotor.setPower(motorOn ? 1.0 : 0.0);

        // Telemetry
        telemetry.addData("Motor Power", testMotor.getPower());
        telemetry.addData("Motor State", motorOn ? "ON" : "OFF");
        telemetry.addLine("Press 'A' to toggle motor");
        telemetry.update();
    }
}
