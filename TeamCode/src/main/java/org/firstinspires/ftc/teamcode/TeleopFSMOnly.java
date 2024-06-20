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
@TeleOp (name = "TeleopFSMOnly")
public class TeleopFSMOnly extends OpMode {

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
    public Servo LSSLeft;
    public Servo LSSRight;
    public Servo rotate;
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightBack;


    ElapsedTime liftTimer = new ElapsedTime();
    final double claw_open = 0.5;
    final double claw_close = 1;
    final double intake_down = 0;
    final double intake_up = 0.775;
    final double arm_down = 0.5;
    final double lock_servo = 0;

    final double close_time = 5;

    final int back_motor = 200; // the ecoder position when the intake moves back;




    public void init() {
        liftTimer.reset();
        LS_MIDDLE = hardwareMap.get(DcMotor.class,"middleLSM");
        Claw_Right = hardwareMap.get(Servo.class,"clawRight");
        Claw_Left = hardwareMap.get(Servo.class, "clawLeft");
        Arm = hardwareMap.get(Servo.class,"arm");
        IntakeRight = hardwareMap.get(Servo.class, "intakeLeft");
        IntakeLeft = hardwareMap.get(Servo.class, "intakeRight");
        LockLeft=hardwareMap.get(Servo.class, "lockLeft");
        LockRight=hardwareMap.get(Servo.class,"lockRight");
        LS_MIDDLE.setTargetPosition(back_motor);
        LS_MIDDLE.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Claw_Left.setDirection(Servo.Direction.REVERSE);
        IntakeLeft.setDirection(Servo.Direction.REVERSE);
        LockLeft.setDirection(Servo.Direction.REVERSE);
        rotate = hardwareMap.get(Servo.class, "rotate");
        LSSLeft = hardwareMap.get(Servo.class,"LSSLeft");
        LSSRight = hardwareMap.get(Servo.class, "LSSRight");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");


    }


    public void loop() {
        LS_MIDDLE.setPower(1.0);
        switch (states){
            case START:
                if (gamepad1.a){
                    Claw_Left.setPosition(claw_close);
                    Claw_Right.setPosition(claw_close);
                    states=States.INTAKE_UP;
                }
                break;

            case INTAKE_UP:
                if (Claw_Right.getPosition()==claw_close){
                    IntakeRight.setPosition(intake_up);
                    IntakeLeft.setPosition(intake_up);
                    states = States.LS_MIDDLE_BACK;
                }
                break;

            case LS_MIDDLE_BACK:
                if (IntakeRight.getPosition() == intake_up){
                    LS_MIDDLE.setTargetPosition(back_motor);
                    states = States.ARM_DOWN;
                }
                break;

            case ARM_DOWN:
                if (Math.abs(LS_MIDDLE.getCurrentPosition()-back_motor)<10){
                    Arm.setPosition(arm_down);
                    states = States.LOCK_SERVO;
                }
                break;

            case LOCK_SERVO:
                if (Arm.getPosition()==arm_down){
                    LockLeft.setPosition(lock_servo);
                    LockRight.setPosition(lock_servo);
                    states = States.CLAW_OPEN;
                }
                break;

            case CLAW_OPEN:

                if (LockRight.getPosition()== lock_servo){
                    Claw_Left.setPosition(claw_open);
                    Claw_Right.setPosition(claw_close);
                }
                break;

        }



    }
}
