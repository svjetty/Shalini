package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Gripper2")
public class GripperWheel extends LinearOpMode {
    CRServo gripperRightServo;
    @Override
    public void runOpMode() throws InterruptedException {
        Servo gripperLeftServo = hardwareMap.servo.get("gripperLeftServo");
        CRServo gripperRightServo = hardwareMap.crservo.get("gripperRightServo");
        waitForStart();
        while (opModeIsActive()) {

            while (gamepad1.b) {
                gripperRightServo.setPower(1);
            }
            while (gamepad1.x) {
                gripperRightServo.setPower(0);
            }
            while (gamepad1.y) {
                gripperLeftServo.setPosition(0);
            }
            while (gamepad1.a) {
                gripperLeftServo.setPosition(0.35);
            }
        }

    }
}
