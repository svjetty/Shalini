
package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "April Tag 13 Detection", group = "Limelight")
public class AprilTag13DetectionOnly extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Ensure pipeline 3 is for AprilTag detection
        limelight.pipelineSwitch(3);

        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                boolean tag13Found = false;

                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (fr.getFiducialId() == 13) {
                        tag13Found = true;

                        telemetry.addData("AprilTag 13", "X: %.2f°, Y: %.2f°",
                                fr.getTargetXDegrees(), fr.getTargetYDegrees());

                        Pose3D botpose = result.getBotpose();
                        telemetry.addData("Botpose", botpose.toString());
                        break;  // No need to check others
                    }
                }

                if (!tag13Found) {
                    telemetry.addData("AprilTag 13", "Not visible");
                }
            } else {
                telemetry.addData("Limelight", "No valid data");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
