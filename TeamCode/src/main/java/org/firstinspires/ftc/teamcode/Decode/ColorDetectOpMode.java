package org.firstinspires.ftc.teamcode.Decode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Detection (OpMode)")
public class ColorDetectOpMode extends OpMode {

    private NormalizedColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        telemetry.addLine("Color sensor initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Read color
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;

        String detectedColor = detectColor(r, g, b);

        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);
        telemetry.addData("Detected", detectedColor);
        telemetry.update();
    }

    private String detectColor(float r, float g, float b) {

        float total = r + g + b;
        if (total == 0) return "UNKNOWN"; // avoid divide-by-zero

        float rn = r / total;
        float gn = g / total;
        float bn = b / total;

        // GREEN detection
        if (gn > 0.45 && gn > rn + 0.15 && gn > bn + 0.15) {
            return "GREEN";
        }

        // PURPLE detection
        if ((rn > 0.35 && bn > 0.35) && gn < 0.30) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}
