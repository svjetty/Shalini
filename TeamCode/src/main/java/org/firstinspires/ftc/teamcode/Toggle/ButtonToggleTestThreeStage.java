package org.firstinspires.ftc.teamcode.Toggle;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Button Toggle Test - 3 Stage", group="Toggle")
public class ButtonToggleTestThreeStage extends OpMode {

    private DcMotor testMotor;
    private int speedState = 0;  // 0 = OFF, 1 = LOW, 2 = HIGH
    private boolean aPressedLast = false;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        telemetry.addLine("=== 3-Speed Motor Toggle Test ===");
        telemetry.addLine("Press 'A' on Gamepad1 to cycle motor speed:");
        telemetry.addLine("OFF → LOW → HIGH → OFF ...");
        telemetry.addLine("Motor config name: 'testMotor'");
        telemetry.addLine("----------------------------------");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;

        // Rising edge detection for toggle
        if (aPressed && !aPressedLast) {
            speedState = (speedState + 1) % 3;  // Cycle through 0 → 1 → 2 → 0
        }
        aPressedLast = aPressed;

        // Set motor power based on speed state
        double power = 0.0;
        switch (speedState) {
            case 1: power = 0.3; break; // Low speed
            case 2: power = 1.0; break; // High speed
        }
        testMotor.setPower(power);

        // Telemetry output
        telemetry.addData("Current Speed Mode", speedState);
        telemetry.addData("Motor Power", power);
        telemetry.addLine("Press 'A' to cycle: OFF → LOW → HIGH");
        telemetry.update();
    }
}
