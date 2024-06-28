package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "intakeautomation")
public class Intakeautomation extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        DcMotor rollers = hardwareMap.get(DcMotor.class, "rollers");
        CRServo ramp = hardwareMap.get(CRServo.class, "ramp");

        intakeLeft.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            rollers.setPower(gamepad2.left_stick_y * 0.6);
            ramp.setPower(gamepad2.left_stick_y);

            if (gamepad2.left_stick_y != 0) {
                intakeLeft.setPosition(0);
                intakeRight.setPosition(0);
            } else {
                intakeLeft.setPosition(0.268);
                intakeRight.setPosition(0.268);
            }


        }
    }
}

