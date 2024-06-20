package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Suspension")
public class SuspensionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here


        DcMotor zero = hardwareMap.get(DcMotor.class, "leftLSM");
        DcMotor one = hardwareMap.get(DcMotor.class, "rightLSM");

        //CRServo zeros = hardwareMap.get(CRServo.class, "suspensionLeft");
        //CRServo ones = hardwareMap.get(CRServo.class, "suspensionRight");

        zero.setDirection(DcMotorSimple.Direction.REVERSE);
        //ones.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {

            zero.setPower(-gamepad1.right_stick_y*0.75);
            one.setPower(-gamepad1.right_stick_y*0.75);

            }
        }
    }
