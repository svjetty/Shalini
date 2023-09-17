package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp (name = "Drone Launch")
public class DroneLaunch extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo droneLeft = hardwareMap.get(CRServo.class, "droneLeft");
        CRServo droneRight = hardwareMap.get(CRServo.class, "droneRight");
        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.left_bumper) {
                droneRight.setPower(1);
                droneLeft.setPower(-1);
            }

            else if (gamepad1.right_bumper) {
                droneRight.setPower(-1);
                droneLeft.setPower(1);
            }

            else if (gamepad1.a) {
                    droneRight.setPower(0);
                    droneLeft.setPower(0);
                }
            }
        }
    }

