package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.graphics.Color;

@TeleOp(name = "Color Sensor Test", group = "Test")
public class TestColorSensor extends OpMode {

    RevColorSensorV3 revSensor;     // Specific class (for reference)
    ColorSensor colorSensor;        // General interface for LED control

    @Override
    public void init() {
        // Access the color sensor from hardware
        revSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Turn on the built-in LED
        try {
            colorSensor.enableLed(true);
            telemetry.addData("LED", "Enabled");
        } catch (Exception e) {
            telemetry.addData("LED", "Could not enable (not supported)");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Move colored objects in front of the sensor.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get raw color values
        int red = revSensor.red();
        int green = revSensor.green();
        int blue = revSensor.blue();
        int alpha = revSensor.alpha();

        // Convert RGB to HSV (scale RGB from 0–255)
        float[] hsvValues = new float[3];
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsvValues); // scaling factor: 8

        // Show telemetry data
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);
        telemetry.addData("Alpha (Clear)", alpha);
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);

        telemetry.update();
    }
}
