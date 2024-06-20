package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "Button Toggle Test")
public class ButtonToggleTest extends LinearOpMode {
    int x = 0;
    public void execute(){
        telemetry.addData("Task is running for 1 sec",x);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        Servo IntakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        boolean togglea = true;



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a) {
              togglea =!togglea;
            }

            if (togglea){
                IntakeLeft.setPosition(0);
            } else{
                IntakeLeft.setPosition(1);
            }
        }
    }
}



