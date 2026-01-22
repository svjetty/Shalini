package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Dual Sensor Ball Detector")
public class DualBallDetector extends OpMode {

    // Left sensor
    private NormalizedColorSensor colorLeft;
    private DistanceSensor distanceLeft;

    // Right sensor
    private NormalizedColorSensor colorRight;
    private DistanceSensor distanceRight;

    // distance threshold in cm
    private static final double BALL_DETECT_CM = 5.0;

    // Store hue for debugging
    private float hueLeft = 0;
    private float hueRight = 0;

    @Override
    public void init() {

        // Left sensor
        colorLeft = hardwareMap.get(NormalizedColorSensor.class, "colorLeft");
        distanceLeft = hardwareMap.get(DistanceSensor.class, "colorLeft");

        // Right sensor
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "colorRight");
        distanceRight = hardwareMap.get(DistanceSensor.class, "colorRight");

        telemetry.addLine("Dual Color Sensors Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Read distances
        double distL = distanceLeft.getDistance(DistanceUnit.CM);
        double distR = distanceRight.getDistance(DistanceUnit.CM);

        boolean leftHasBall = distL < BALL_DETECT_CM;
        boolean rightHasBall = distR < BALL_DETECT_CM;

        telemetry.addData("Left cm", distL);
        telemetry.addData("Right cm", distR);
        telemetry.addData("Left Ball?", leftHasBall);
        telemetry.addData("Right Ball?", rightHasBall);

        String detectedColor = "NONE";

        // FIRST check LEFT sensor
        if (leftHasBall) {
            detectedColor = detectColorLeft();
        }

        // If left is UNKNOWN but right has ball → check right
        if (detectedColor.equals("UNKNOWN") || detectedColor.equals("NONE")) {
            if (rightHasBall) {
                detectedColor = detectColorRight();
            }
        }

        telemetry.addData("Detected Color", detectedColor);

        // Show hue for debugging
        telemetry.addData("Left Hue", hueLeft);
        telemetry.addData("Right Hue", hueRight);

        telemetry.update();
    }


    // ----------------------
    // LEFT SENSOR DETECTION
    // ----------------------
    private String detectColorLeft() {
        NormalizedRGBA c = colorLeft.getNormalizedColors();

        float r = c.red, g = c.green, b = c.blue;
        return classifyColor(r, g, b, true);
    }

    // -----------------------
    // RIGHT SENSOR DETECTION
    // -----------------------
    private String detectColorRight() {
        NormalizedRGBA c = colorRight.getNormalizedColors();

        float r = c.red, g = c.green, b = c.blue;
        return classifyColor(r, g, b, false);
    }


    // ----------------------
    // SHARED COLOR LOGIC
    // ----------------------
    private String classifyColor(float r, float g, float b, boolean isLeft) {

        float total = r + g + b;
        if (total <= 0) return "UNKNOWN";

        float rn = r / total;
        float gn = g / total;
        float bn = b / total;

        float[] hsv = new float[3];

        android.graphics.Color.RGBToHSV(
                (int)(rn * 255),
                (int)(gn * 255),
                (int)(bn * 255),
                hsv
        );

        float hue = hsv[0];

        if (isLeft) hueLeft = hue;
        else hueRight = hue;

        // GREEN range
        if (hue > 80 && hue < 160) {
            return "GREEN";
        }

        // PURPLE range (wide & tested)
        if (hue > 200 && hue < 300) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}
