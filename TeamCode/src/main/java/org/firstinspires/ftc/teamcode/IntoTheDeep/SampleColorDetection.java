package org.firstinspires.ftc.teamcode.IntoTheDeep;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.graphics.Color;

@TeleOp(name = "Sample Colour Detection", group = "IntoTheDeep")
public class SampleColorDetection extends OpMode {

    RevColorSensorV3 revSensor;
    ColorSensor colorSensor;

    @Override
    public void init() {
        // Get both sensor references
        revSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Turn on LED
        try {
            colorSensor.enableLed(true);
            telemetry.addData("LED", "ON");
        } catch (Exception e) {
            telemetry.addData("LED", "Failed to enable");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get raw RGB values
        int red = revSensor.red();
        int green = revSensor.green();
        int blue = revSensor.blue();

        // Convert to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv); // Scale 0-255

        String detectedColor = getDetectedColor(hsv);

        telemetry.addData("Raw Red", red);
        telemetry.addData("Raw Green", green);
        telemetry.addData("Raw Blue", blue);
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.update();
    }

    // Helper method to determine color based on hue
    private String getDetectedColor(float[] hsv) {
        float hue = hsv[0];

        if (hue >= 0 && hue < 30) {
            return "Red";
        } else if (hue >= 40 && hue < 70) {
            return "Yellow";
        } else if (hue >= 200 && hue < 250) {
            return "Blue";
        } else {
            return "Unknown";
        }
    }
}
