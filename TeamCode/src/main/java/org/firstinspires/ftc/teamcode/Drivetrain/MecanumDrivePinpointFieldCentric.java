package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive - Pinpoint Field Centric", group = "Drivetrain")
public class MecanumDrivePinpointFieldCentric extends OpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    private boolean slowMode = false;
    private boolean lastBumperState = false;

    @Override
    public void init() {

        // --- Motors ---
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor  = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor  = hardwareMap.dcMotor.get("rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Pinpoint IMU ---
        // Make sure in Robot Config the Pinpoint IMU device name is "pinpointIMU"
        imu = hardwareMap.get(IMU.class, "pinpointIMU");

        // Pinpoint board is usually mounted flat.
        // If you mounted differently, we can change these directions.
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void loop() {

        telemetry.addLine("Dpad Down = Reset Heading");
        telemetry.addLine("Left Bumper = Toggle Slow Mode");

        // --- Slow mode toggle ---
        boolean currentBumperState = gamepad1.left_bumper;
        if (currentBumperState && !lastBumperState) {
            slowMode = !slowMode;
        }
        lastBumperState = currentBumperState;

        double speedMultiplier = slowMode ? 0.4 : 1.0;

        // --- Gamepad input ---
        double y = -gamepad1.left_stick_y * speedMultiplier;
        double x =  gamepad1.left_stick_x * speedMultiplier;
        double rx = gamepad1.right_stick_x * speedMultiplier;

        // --- Reset heading ---
        if (gamepad1.dpad_down) {
            imu.resetYaw();
        }

        // --- Read current robot heading ---
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // --- Field centric transform ---
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Counteract imperfect strafing
        rotX *= 1.1;

        // --- Mecanum power calculations ---
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower  = (rotY + rotX + rx) / denominator;
        double backLeftPower   = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower  = (rotY + rotX - rx) / denominator;

        // --- Set motor powers ---
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        // --- Telemetry ---
        telemetry.addData("Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Heading (deg)",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }
}
