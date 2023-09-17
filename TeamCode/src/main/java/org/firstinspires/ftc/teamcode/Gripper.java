package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Gripper")
public class Gripper extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo gripperLeftServo= hardwareMap.get(Servo.class, "gripperLeftServo");
        Servo gripperRightServo= hardwareMap.get(Servo.class, "gripperRightServo");

        gripperRightServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            while (gamepad1.x) {
                gripperLeftServo.setPosition(0);
                gripperRightServo.setPosition(1);}

            while (gamepad1.y){
                gripperLeftServo.setPosition(0);
                gripperRightServo.setPosition(0);}

            while (gamepad1.b){
                gripperLeftServo.setPosition(1);
                gripperRightServo.setPosition(1);}

            while (gamepad1.a){
                gripperRightServo.setPosition(0.35);
                gripperLeftServo.setPosition(0.35);}
            }

        }
    }

