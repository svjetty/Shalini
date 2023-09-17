package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Beta")
public class Beta extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor armLeft = hardwareMap.dcMotor.get("armLeft");
        DcMotor armRight = hardwareMap.dcMotor.get("armRight");
        CRServo duck = hardwareMap.crservo.get("duck");
        Servo gripperLeft = hardwareMap.servo.get("gripperLeft");
        Servo gripperRight = hardwareMap.servo.get("gripperRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        gripperLeft.setPosition(0.75);
        gripperRight.setPosition(0.6);

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            //Arm Up and Down
            double arm_speed = gamepad2.left_stick_y*0.7;
            armLeft.setPower(arm_speed);
            armRight.setPower(arm_speed*-1);

            //Carousel Duck Rotation
            if (gamepad2.x) {
                duck.setPower(1);
            } else if (gamepad2.b) {
                duck.setPower(-1);
            } else if (gamepad2.a) {
                duck.setPower(0);
            }

            //Gripper - Pick Up Cubes
            while (gamepad2.left_bumper==true) {
                gripperRight.setPosition(0.75);
                gripperLeft.setPosition(0.5);
            }

            //Gripper - Drop Cubes
            while (gamepad2.right_bumper==true) {
                gripperRight.setPosition(0.6);
                gripperLeft.setPosition(0.75);
            }


                }
            }
            }




