package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Turret Follow AprilTag 13", group = "Limelight")
public class TurretFollowAprilTag13 extends OpMode {

    private Limelight3A limelight;
    private DcMotor turret;

    // Control constants (tune these)
    private static final double KP = 0.02;
    private static final double MIN_POWER = 0.05;
    private static final double MAX_POWER = 0.5;
    private static final double DEADZONE = 0.5; // degrees

    @Override
    public void init() {
        // Hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Start Limelight AprilTag pipeline
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addLine("Turret Follow AprilTag 13 - INIT complete");
    }

    @Override
    public void start() {
        // (Optional) Any reset or calibration code here
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            boolean tagFound = false;

            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (fr.getFiducialId() == 13) {
                    tagFound = true;

                    double tx = fr.getTargetXDegrees();
                    double turretPower = 0;

                    if (Math.abs(tx) > DEADZONE) {
                        turretPower = KP * tx;

                        if (Math.abs(turretPower) < MIN_POWER) {
                            turretPower = Math.signum(turretPower) * MIN_POWER;
                        }

                        turretPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, turretPower));
                    }

                    // Negate turret power if needed based on motor direction
                    turret.setPower(-turretPower);

                    telemetry.addData("AprilTag 13", "tx=%.2f°", tx);
                    telemetry.addData("Turret Power", "%.2f", turretPower);

                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("Botpose", botpose.toString());
                    break;
                }
            }

            if (!tagFound) {
                telemetry.addLine("AprilTag 13 not visible");
                turret.setPower(0);
            }

        } else {
            telemetry.addLine("No valid Limelight data");
            turret.setPower(0);
        }

        LLStatus status = limelight.getStatus();
        telemetry.addData("FPS", status.getFps());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
        limelight.stop();
    }
}
