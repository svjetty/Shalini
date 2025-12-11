package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test - Button", group = "Test")
public class TestMotorButton extends OpMode {

    private DcMotor testMotor;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor"); // Replace with your motor's config name

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press A to run motor forward");
        telemetry.addLine("Press Y to run motor in reverse");
        telemetry.addLine("Release to stop");
        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Press A to run motor forward");
        telemetry.addLine("Press Y to run motor in reverse");
        telemetry.addLine("Release to stop");

        double power = 0.0;

        if (gamepad1.a) {
            power = 0.5;
        } else if (gamepad1.y) {
            power = -0.5;
        }

        testMotor.setPower(power);

        telemetry.addData("Motor Power", power);
        telemetry.update();
    }
}
