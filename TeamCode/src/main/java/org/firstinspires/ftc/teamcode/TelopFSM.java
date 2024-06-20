package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "TeleopFSM")
public class TelopFSM extends LinearOpMode {

    public enum States{
        START,
        CLAW_CLOSE,
        INTAKE_UP,
        LS_MIDDLE_BACK,
        ARM_DOWN,
        LOCK_SERVO,
        CLAW_OPEN,

    }

    States states = States.START;

    ElapsedTime liftTime = new ElapsedTime();
    final double claw_open = 0.5;
    final double claw_close = 1;
    final double intake_down = 0;
    final double intake_up = 0.775;
    final double arm_down = 0.5;
    final double lock_servo = 0;

    final double claw_close_time = 5;



    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        Servo intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        Servo intakeRight = hardwareMap.get(Servo.class,"intakeRight");
        Servo clawLeft = hardwareMap.get(Servo.class, "clawRight");
        Servo clawRight = hardwareMap.get(Servo.class, "clawLeft");
        Servo LSSLeft =  hardwareMap.get(Servo.class,"LSSLeft");
        Servo LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        Servo arm = hardwareMap.get(Servo.class, "arm");
        Servo rotate = hardwareMap.get(Servo.class, "rotate");
        boolean togglea = true;
        boolean toggleb = true;
        boolean toggley = true;

        //lockLeft.setDirection(Servo.Direction.REVERSE);
        LSSLeft.setDirection(Servo.Direction.REVERSE);
        //suspensionLeft.setDirection(Servo.Direction.REVERSE);
        intakeLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);
        rotate.setPosition(0);


        //DcMotor rightLSM = hardwareMap.get(DcMotor.class,"rightLSM");
        //DcMotor leftLSM = hardwareMap.get(DcMotor.class, "leftLSM");
        DcMotor middleLSM = hardwareMap.get(DcMotor.class, "middleLSM");
        //rightLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleLSM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftLSM.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        double drive, turn, strafe;

        double fleftpower, frightpower, bleftpower, brightpower;
        double rightsidepower, leftsidepower;


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        waitForStart();

        if (isStopRequested()) return;



        while(opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            //rightLSM.setPower(gamepad2.right_stick_y);
            //leftLSM.setPower(gamepad2.right_stick_y);
            middleLSM.setPower(gamepad2.left_stick_y);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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

            rightFront.setPower(frontRightPower);
            leftFront.setPower(frontLeftPower);
            rightBack.setPower(backRightPower);
            leftBack.setPower(backLeftPower);




        }

    }
}

