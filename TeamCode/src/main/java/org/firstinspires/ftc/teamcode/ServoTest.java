package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Servo Test")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here


        CRServo zero = hardwareMap.get(CRServo.class, "zero");
        CRServo one = hardwareMap.get(CRServo.class, "one");
        Servo two = hardwareMap.get(Servo.class, "two");
        Servo three = hardwareMap.get(Servo.class, "three");

        one.setDirection(CRServo.Direction.REVERSE);
        three.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {

            zero.setPower(-gamepad1.left_stick_y);
            one.setPower(-gamepad1.left_stick_y);

            if (gamepad1.y){
                two.setPosition(0);
                three.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {
                two.setPosition(0.5);
                three.setPosition(0.5);

            } else if (gamepad1.a) {
                two.setPosition(1);
                three.setPosition(1);
            }
        }
    }
}
