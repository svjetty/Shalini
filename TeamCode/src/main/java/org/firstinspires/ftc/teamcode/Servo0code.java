package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;

@TeleOp (name = "Servo0code")
public class Servo0code extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo lockLeft;
        //lockLeft = hardwareMap.get(Servo.class, "lockLeft");
        Servo lockRight;
        //lockRight = hardwareMap.get(Servo.class, "lockRight");
        Servo rotate;
        rotate = hardwareMap.get(Servo.class, "rotate");
        Servo arm;
        arm = hardwareMap.get(Servo.class, "arm");
        Servo LSSLeft;
        LSSLeft = hardwareMap.get(Servo.class, "LSSLeft");
        Servo LSSRight;
        LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        //Servo suspensionLeft;
        //suspensionLeft = hardwareMap.get(Servo.class, "suspensionLeft");
        //Servo suspensionRight;
        //suspensionRight = hardwareMap.get(Servo.class, "suspensionRight");
        Servo intakeLeft;
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight;
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        Servo clawLeft;
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        Servo clawRight;
        clawRight = hardwareMap.get(Servo.class, "clawRight");

        //lockLeft.setDirection(Servo.Direction.REVERSE);
        //lockRight.setDirection(Servo.Direction.REVERSE);
        LSSLeft.setDirection(Servo.Direction.REVERSE);
        //suspensionLeft.setDirection(Servo.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        clawLeft.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                //lockLeft.setPosition(1);
                //lockRight.setPosition(1);
                rotate.setPosition(0);
                arm.setPosition(1);
                LSSLeft.setPosition(1);
                LSSRight.setPosition(1);
                //suspensionLeft.setPosition(1);
                //suspensionRight.setPosition(1);
                intakeLeft.setPosition(1);
                intakeRight.setPosition(1);
                clawLeft.setPosition(1);
                clawRight.setPosition(1);
            } else if (gamepad1.x || gamepad1.b) {
                //lockLeft.setPosition(0.5);
                //lockRight.setPosition(0.5);
                rotate.setPosition(0);
                arm.setPosition(0.5);
                LSSLeft.setPosition(0.5);
                LSSRight.setPosition(0.5);
                //suspensionLeft.setPosition(0.5);
                //suspensionRight.setPosition(0.5);
                intakeLeft.setPosition(0.5);
                intakeRight.setPosition(0.5);
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.5);
            } else if (gamepad1.y) {
                //lockLeft.setPosition(0);
                //lockRight.setPosition(0);
                rotate.setPosition(0);
                arm.setPosition(0);
                LSSLeft.setPosition(0);
                LSSRight.setPosition(0);
                //suspensionLeft.setPosition(0);
                //suspensionRight.setPosition(0);
                intakeLeft.setPosition(0);
                intakeRight.setPosition(0);
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.5);
            }


            if (gamepad1.b){

                telemetry.addData("this is the servo position", clawRight.getPosition());
                clawRight.setPosition(1);
                if (clawRight.getPosition()==1){
                    intakeLeft.setPosition(1);
                }
            }
        }
    }
}