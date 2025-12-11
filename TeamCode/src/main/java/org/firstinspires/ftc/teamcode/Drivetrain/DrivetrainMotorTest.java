package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drivetrain Motor Test", group = "Drivetrain")
public class DrivetrainMotorTest extends LinearOpMode {

    DcMotor leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {

        // Map motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Set direction so forward is consistent
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Press START to begin motor test");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Stop all motors first
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);

            // Test each motor with a different button
            if (gamepad1.a) {
                leftFront.setPower(1);
                telemetry.addLine("Testing Left Front Motor");
            }
            else if (gamepad1.b) {
                rightFront.setPower(1);
                telemetry.addLine("Testing Right Front Motor");
            }
            else if (gamepad1.x) {
                leftBack.setPower(1);
                telemetry.addLine("Testing Left Back Motor");
            }
            else if (gamepad1.y) {
                rightBack.setPower(1);
                telemetry.addLine("Testing Right Back Motor");
            }
            else {
                telemetry.addLine("Press A/B/X/Y to test motors");
            }

            telemetry.update();
        }
    }
}
