package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive - Field Centric", group = "Drivetrain")
public class MecanumDriveFieldCentric extends OpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU imu;

    private boolean slowMode = false;
    private boolean lastBumperState = false;

    @Override
    public void init() {
        // Motor initialization
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightBack");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }

    @Override
    public void loop() {

        telemetry.addLine("Press Dpad Down to reset heading");
        telemetry.addLine("Press LB to toggle speed");

        // Speed toggle with debounce
        boolean currentBumperState = gamepad1.left_bumper;
        if (currentBumperState && !lastBumperState) {
            slowMode = !slowMode;
        }
        lastBumperState = currentBumperState;

        double speedMultiplier = slowMode ? 0.4 : 1.0;

        double y = -gamepad1.left_stick_y * speedMultiplier;
        double x = gamepad1.left_stick_x * speedMultiplier;
        double rx = gamepad1.right_stick_x * speedMultiplier;

        if (gamepad1.dpad_down) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        //Telemetry
        telemetry.addData("Speed Mode", slowMode ? "SLOW" : "NORMAL");
        telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
        telemetry.update();
    }
}
