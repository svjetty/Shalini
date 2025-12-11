package org.firstinspires.ftc.teamcode.IntoTheDeep;

import android.graphics.Color;
import android.widget.GridLayout;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp (name = "FSM_Code_Blue", group = "IntoTheDeep")
public class FSM_Code_Blue extends OpMode {
    //MOTORS
    public DcMotor leftBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;
    public DcMotor LSMotor = null;
    public DcMotor intakeMotor = null;

    //SERVOS
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    public Servo sampleServo = null;
    public Servo leftOuttakeServo = null;
    public Servo rightOuttakeServo = null;
    public Servo clawServo = null;
    public CRServo rightHLSServo = null;
    public CRServo leftLHSServo = null;
    //SENSORS
    public IMU imu = null;
    public NormalizedColorSensor colorSensor = null;
    public DistanceSensor leftDistanceSensor = null;
    public DistanceSensor rightDistanceSensor = null;

    //FSM INTAKE
    public enum Intake {
        DownToPick,
        ColorSensor,
        BlueIntakeUp,
        Transfer,
    }

    //FSM OUTTAKE - SAMPLE
    public enum Sample {
        IntakeDownSample,
        LSUpSample,
        ArmOutSample,
        ClawOpenSample,
        ArmResetSample,
        LSDownSample,
    }

    //FSM OUTTAKE - SPECIMEN
    public enum Specimen {
        IntakeDownSpecimen,
        ArmOutSpecimen,
        ClawOpenSpecimen,
        ArmRestSpecimen,
    }

    Intake intake = Intake.DownToPick;
    Sample sample = Sample.IntakeDownSample;
    Specimen specimen = Specimen.IntakeDownSpecimen;
    ElapsedTime intake_timer = new ElapsedTime();
    final double close_time = 1;
    final double hang = 2;

    @Override
    public void init() {

        //IMU LOCALIZATION
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();

        //MECANUM DRIVE MOTORS MAPPING
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        //MECANUM DRIVE MOTORS DIRECTION REVERESE
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //MECANUM WHEELS MOTOR BRAKE FUNCTION
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //VERTICAL LINEAR SLIDE MOTOR MAPPING, SET ENCODER MODE & RESET ENCODERS
        LSMotor = hardwareMap.get(DcMotor.class, "LSMotor");
        LSMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LSMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // HORIZONTAL LINEAR SLIDES SERVOS MAPPING AND DIRECTION REVERSE
        rightHLSServo = hardwareMap.get(CRServo.class, "rightHLSServo");
        leftLHSServo = hardwareMap.get(CRServo.class, "leftLHSServo");
        rightHLSServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //INTAKE MOTOR MAPPING
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //INTAKE SERVOS MAPPING AND DIRECTION REVERSE
        leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        rightIntakeServo.setDirection(Servo.Direction.REVERSE);

        //OUTTAKE SERVOS MAPPING AND DIRECTION REVERSE
        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo");
        rightOuttakeServo.setDirection(Servo.Direction.REVERSE);

        //OUTTAKE CLAW
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawServo.setDirection(Servo.Direction.REVERSE);

        //COLOUR SENSOR MAPPING
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //DISTANCE SENSOR MAPPING
        leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        //INITIALIZATION SETTINGS
        leftIntakeServo.setPosition(0.38);
        rightIntakeServo.setPosition(0.44);
        sampleServo.setPosition(0);
        leftOuttakeServo.setPosition(0.09);
        rightOuttakeServo.setPosition(0.09);
        clawServo.setPosition(0);
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addLine();
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        double test = 0;
        test = colors.blue;

        double rdistance = rightDistanceSensor.getDistance(DistanceUnit.MM);
        double ldistance = leftDistanceSensor.getDistance(DistanceUnit.MM);


        switch (intake) {

            case DownToPick:
                if (gamepad2.a) {
                    leftIntakeServo.setPosition(0.02);
                    rightIntakeServo.setPosition(0);
                    intake_timer.reset();
                    intake = Intake.ColorSensor;
                }
                break;

            case ColorSensor:
                if (colors.red >= 0.009 && colors.red <= 0.014) {
                    sampleServo.setPosition(0);
                    intake_timer.reset();
                    intake = Intake.DownToPick;
                } else {
                    sampleServo.setPosition(0.5);
                    telemetry.addData("The Color is:", colors.red);
                    intake_timer.reset();
                    intake = Intake.BlueIntakeUp;
                }
                break;

            case BlueIntakeUp:
                if (test >= 0.01 && test <= 0.015) {
                    leftIntakeServo.setPosition(0.38);
                    rightIntakeServo.setPosition(0.44);
                    intake_timer.reset();
                    intake = Intake.Transfer;
                }
                break;

            case Transfer:
                if (rdistance <= 30 && ldistance <= 30) {
                    clawServo.setPosition(0.471);
                    intake_timer.reset();
                    intake = Intake.DownToPick;
                }
                break;

            default:
                intake = Intake.DownToPick;
        }
        switch (sample) {
            case IntakeDownSample:
                if (gamepad2.left_bumper) {
                    leftIntakeServo.setPosition(0.02);
                    rightIntakeServo.setPosition(0.0);
                    intake_timer.reset();
                    sample = Sample.LSUpSample;
                }
                break;

            case LSUpSample:
                if (intake_timer.seconds() >= close_time) {
                    LSMotor.setTargetPosition(3250);
                    LSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSMotor.setPower(1);
                    leftIntakeServo.setPosition(0.38);
                    rightIntakeServo.setPosition(0.44);
                    intake_timer.reset();
                    sample = Sample.ArmOutSample;
                }
                break;

            case ArmOutSample:
                if (intake_timer.seconds() >= close_time) {
                    leftOuttakeServo.setPosition(0.645);
                    rightOuttakeServo.setPosition(0.645);
                    intake_timer.reset();
                    sample = Sample.ClawOpenSample;
                }
                break;

            case ClawOpenSample:
                if (gamepad2.b) {
                    clawServo.setPosition(0);
                    intake_timer.reset();
                    sample = Sample.ArmResetSample;
                }
                break;

            case ArmResetSample:
                if (intake_timer.seconds() >= close_time) {
                    leftOuttakeServo.setPosition(0.09);
                    rightOuttakeServo.setPosition(0.09);
                    intake_timer.reset();
                    sample = Sample.LSDownSample;
                }
                break;

            case LSDownSample:
                if (intake_timer.seconds() >= close_time) {
                    LSMotor.setTargetPosition(0);
                    LSMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LSMotor.setPower(1);
                    intake_timer.reset();
                    sample = Sample.IntakeDownSample;
                }
                break;
            default:
                sample = Sample.IntakeDownSample;
        }

        switch (specimen) {
            case IntakeDownSpecimen:
                if (gamepad2.right_bumper) {
                    leftIntakeServo.setPosition(0.02);
                    rightIntakeServo.setPosition(0.0);
                    intake_timer.reset();
                    specimen = Specimen.ArmOutSpecimen;
                }
                break;

            case ArmOutSpecimen:
                if (intake_timer.seconds() >= close_time) {
                    leftOuttakeServo.setPosition(0.645);
                    rightOuttakeServo.setPosition(0.645);
                    intake_timer.reset();
                    specimen = Specimen.ClawOpenSpecimen;
                }
                break;

            case ClawOpenSpecimen:
                if (gamepad2.b) {
                    clawServo.setPosition(0);
                    leftIntakeServo.setPosition(0.38);
                    rightIntakeServo.setPosition(0.44);
                    intake_timer.reset();
                    specimen = Specimen.ArmRestSpecimen;
                }
                break;

            case ArmRestSpecimen:
                if (intake_timer.seconds() >= close_time) {
                    leftOuttakeServo.setPosition(0.09);
                    rightOuttakeServo.setPosition(0.09);
                    intake_timer.reset();
                    specimen = Specimen.IntakeDownSpecimen;
                }
                break;

            default: specimen = Specimen.IntakeDownSpecimen;
        }

        /*
        if (gamepad2.y && intake != Intake.DownToPick) {
            intake = Intake.DownToPick;
        }

        if (gamepad2.y && sample != Sample.IntakeDownSample) {
            sample = Sample.IntakeDownSample;
        }

        if (gamepad2.y && specimen != Specimen.IntakeDownSpecimen) {
            specimen = Specimen.IntakeDownSpecimen;
        }
        */

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // FIELD CENTRIC DRIVE RESET
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
        rightFront.setPower(frontRightPower * 0.7);
        leftFront.setPower(frontLeftPower * 0.7);
        rightBack.setPower(backRightPower * 0.7);
        leftBack.setPower(backLeftPower * 0.7);

        // HORIZONTAL LINEAR SLIDES MOVEMENT
        rightHLSServo.setPower(-gamepad2.left_stick_y * 0.5);
        leftLHSServo.setPower(-gamepad2.left_stick_y * 0.5);

        // INTAKE ROLLERS
        intakeMotor.setPower(gamepad2.right_stick_y);

    }
}

