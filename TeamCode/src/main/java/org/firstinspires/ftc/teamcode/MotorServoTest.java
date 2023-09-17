package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Motor_Servo_Test")
public class MotorServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here

        DcMotor testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        Servo testServo = hardwareMap.get(Servo.class, "testServo");

        testServo.setPosition(0.5);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {

            double leftx = gamepad1.left_stick_x;
            double lefty = gamepad1.left_stick_y;
            double rightx = gamepad1.right_stick_x;
            double righty = gamepad1.right_stick_y;

            double speed = leftx + lefty + righty + rightx;

            testMotor.setPower(speed);

            if (gamepad1.y){
                testServo.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {
                testServo.setPosition(0.5);

            } else if (gamepad1.a) {
                testServo.setPosition(1);
            }

        }
    }
}