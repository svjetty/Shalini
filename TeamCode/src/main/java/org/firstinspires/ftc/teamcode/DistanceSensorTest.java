package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Position of the arm when it's lifted
        //int armUpPosition = 1000;

        // Position of the arm when it's down
        int LSDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor LSRight = hardwareMap.dcMotor.get("Arm Motor");
        DcMotor LSLeft = hardwareMap.dcMotor.get("LSLeft");

        // Reset the motor encoder so that it reads zero ticks
        LSRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Sets the starting position of the arm to the down position
        LSRight.setTargetPosition(LSDownPosition);
        LSLeft.setTargetPosition(LSDownPosition);
        LSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.a) {
                DistanceSensor dsensor = hardwareMap.get(DistanceSensor.class,"dsensor");
                double distance = dsensor.getDistance(DistanceUnit.MM);
                double LSdistance = (distance - 150) * 5;
                int ticks = (int) LSdistance;
                if (150 > distance) {
                LSRight.setTargetPosition(ticks);
                LSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LSRight.setPower(0.5);
                LSLeft.setTargetPosition(ticks);
                LSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LSLeft.setPower(0.5);
            } else if (150 < distance){
                    LSRight.setTargetPosition(LSDownPosition);
                    LSRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSRight.setPower(0.5);
                    LSLeft.setTargetPosition(LSDownPosition);
                    LSLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSLeft.setPower(0.5);
                }

            // If the B button is pressed, lower the arm
            //if (gamepad1.b) {
                //armMotor.setTargetPosition(LSDownPosition);
                //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //armMotor.setPower(0.5);
            }

            // Get the current position of the armMotor
            double position = LSRight.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = LSRight.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}

