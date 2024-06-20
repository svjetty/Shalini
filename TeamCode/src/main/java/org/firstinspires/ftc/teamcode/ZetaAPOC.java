package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "Zeta APOC")
public class ZetaAPOC extends LinearOpMode {

    int x = 0;

    public void execute() {telemetry.addData("Task is running for 1 sec", x);}


    @Override
    public void runOpMode() throws InterruptedException {


        // INTAKE ARM, ROLLERS & RAMP MAPPING
        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class,"intakeRight");
        DcMotor rollers = hardwareMap.get(DcMotor.class, "rollers");
        CRServo ramp = hardwareMap.get(CRServo.class, "ramp");

        // INTAKE ARM REVERSE
        intakeLeft.setDirection(Servo.Direction.REVERSE);

        //Servo clawLeft = hardwareMap.get(Servo.class, "clawRight");
        //Servo clawRight = hardwareMap.get(Servo.class, "clawLeft");
        //Servo LSSLeft =  hardwareMap.get(Servo.class,"LSSLeft");
        //Servo LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        //Servo arm = hardwareMap.get(Servo.class, "arm");
        //Servo rotate = hardwareMap.get(Servo.class, "rotate");

        //lockLeft.setDirection(Servo.Direction.REVERSE);
        //LSSLeft.setDirection(Servo.Direction.REVERSE);
        //clawRight.setDirection(Servo.Direction.REVERSE);
        //arm.setDirection(Servo.Direction.REVERSE);

        // LINEAR SLIDES MOTOR MAPPING
        DcMotor rightLSM = hardwareMap.get(DcMotor.class,"rightLSM");
        DcMotor leftLSM = hardwareMap.get(DcMotor.class, "leftLSM");

        // LINEAR SLIDE MOTOR REVERSE
        rightLSM.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollers.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // MECANUM DRIVE MOTORS MAPPING
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        // WHEELS DIRECTION REVERSE
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU MAPPING & ORIENTATION
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();

        // Define GAMEPADS & VARIABLES
        boolean togglea = true;
        boolean toggleb = true;
        boolean toggley = true;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()) {

            // FIELD CENTRIC MECANUM DRIVE CODE
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Field Centric Drive Reset
            if (gamepad1.dpad_down) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // MECANUM DRIVE POWER
            rightFront.setPower(frontRightPower*0.7);
            leftFront.setPower(frontLeftPower*0.7);
            rightBack.setPower(backRightPower*0.7);
            leftBack.setPower(backLeftPower*0.7);

            // LINEAR SLIDE POWER
            rightLSM.setPower(gamepad2.right_stick_y);
            leftLSM.setPower(gamepad2.right_stick_y);

            // RAMP AND ROLLER INTAKE POWER
            rollers.setPower(gamepad2.left_stick_y*0.6);
            ramp.setPower(gamepad2.left_stick_y);

            // INTAKE ARM TOGGLE
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad1.a && !previousGamepad1.a) {
                togglea =!togglea;
            }

            if (togglea){
                intakeLeft.setPosition(0);
                intakeRight.setPosition(0);

            } else {
                intakeLeft.setPosition(0.268);
                intakeRight.setPosition(0.268);
            }




        }

    }
}

