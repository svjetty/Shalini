package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp (name = "TeleopFSM2")
public class TeleopFSM2 extends OpMode{
    public enum States{
        START,
        INTAKE_UP,
        LS_MIDDLE_BACK,
        ARM_DOWN,
        LOCK_SERVO,
        CLAW_OPEN,

    }

    States states = States.START;

    public DcMotor LS_MIDDLE;
    public Servo Claw_Left;
    public Servo Claw_Right;
    public Servo Arm;
    public Servo IntakeLeft;
    public Servo IntakeRight;
    public Servo LockLeft;
    public Servo LockRight;

    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public Servo LSSLeft;
    public Servo LSSRight;
    public Servo rotate;

    ElapsedTime liftTimer = new ElapsedTime();
    final double claw_open = 0.5;
    final double claw_close = 1;
    final double intake_down = 0;
    final double intake_up = 0.775;
    final double arm_down = 0.5;
    final double lock_servo = 0;

    final double close_time = 5;

    final int back_motor = -100; // the encoder position when the intake moves back;

    public void init() {

        liftTimer.reset();
        LS_MIDDLE = hardwareMap.get(DcMotor.class, "middleLSM");
        Claw_Right = hardwareMap.get(Servo.class,"clawRight");
        Claw_Left = hardwareMap.get(Servo.class, "clawLeft");
        Arm = hardwareMap.get(Servo.class,"arm");
        Arm.setDirection(Servo.Direction.REVERSE);
        IntakeRight = hardwareMap.get(Servo.class, "intakeLeft");
        IntakeLeft = hardwareMap.get(Servo.class, "intakeRight");
        LockLeft=hardwareMap.get(Servo.class, "lockLeft");
        LockRight=hardwareMap.get(Servo.class,"lockRight");
        //LS_MIDDLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //LS_MIDDLE.setTargetPosition(back_motor);
        //LS_MIDDLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Claw_Left.setDirection(Servo.Direction.REVERSE);
        IntakeLeft.setDirection(Servo.Direction.REVERSE);
        LockLeft.setDirection(Servo.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        LSSLeft = hardwareMap.get(Servo.class,"LSSLeft");
        LSSLeft.setDirection(Servo.Direction.REVERSE);
        LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        rotate = hardwareMap.get(Servo.class, "rotate");
        rotate.setPosition(0);



    }


    public void loop() {
        //LS_MIDDLE.setPower(1.0);
        switch (states){
            case START:
                if (gamepad1.a){
                    Claw_Left.setPosition(claw_close);
                    Claw_Right.setPosition(claw_close);
                    liftTimer.reset();
                    states= States.INTAKE_UP;
                }
                break;

            case INTAKE_UP:
                if (liftTimer.seconds()>=close_time){
                    IntakeRight.setPosition(intake_up);
                    IntakeLeft.setPosition(intake_up);
                    liftTimer.reset();
                    states = States.LS_MIDDLE_BACK;
                }
                break;

            case LS_MIDDLE_BACK:
                if (liftTimer.seconds()>=close_time){
                    LS_MIDDLE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    LS_MIDDLE.setTargetPosition(back_motor);
                    LS_MIDDLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LS_MIDDLE.setPower(0.3);
                    states = States.ARM_DOWN;
                }
                break;
            case ARM_DOWN:
                if (Math.abs(LS_MIDDLE.getCurrentPosition()-back_motor)<10){
                    Arm.setPosition(arm_down);
                    liftTimer.reset();
                    states = States.LOCK_SERVO;
                }
                break;

            case LOCK_SERVO:
                if (liftTimer.seconds()>=close_time){
                    LockLeft.setPosition(lock_servo);
                    LockRight.setPosition(lock_servo);
                    liftTimer.reset();
                    states = States.CLAW_OPEN;
                }
                break;

            case CLAW_OPEN:

                if (liftTimer.seconds()>=close_time){
                    Claw_Left.setPosition(claw_open);
                    Claw_Right.setPosition(claw_close);
                }
                break;
        }


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
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
