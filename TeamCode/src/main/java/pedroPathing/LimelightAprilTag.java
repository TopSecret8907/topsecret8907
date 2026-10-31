package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp(name = "LimelightAprilTag", group = "Adon")
public class LimelightAprilTag extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        // Get the Limelight from the config
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);

        // Start streaming data
        limelight.start();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);
    }

    private void getFiducialResult(LLResult result, Telemetry telemetry) {
        List<FiducialResult> fiducials = result.getFiducialResults();
        if (result != null && result.isValid()) {
            if (fiducials != null && !fiducials.isEmpty()) {
                // Loop through all detected tags
                for (FiducialResult fr : fiducials) {
                    int tagId = fr.getFiducialId();

                    telemetry.addData("Tag ID", tagId);
                    telemetry.update();
                }
            } else {
                telemetry.addLine("Valid result, but no AprilTags detected.");
                telemetry.update();
            }
        } else {
            telemetry.addLine("No target detected.");
            telemetry.update();
        }
    }


    private void getBarcodeResult(LLResult result, Telemetry telemetry) {
        List<LLResultTypes.BarcodeResult> barcode = result.getBarcodeResults();
        if (result != null && result.isValid()) {
            if (barcode != null && !barcode.isEmpty()) {
                // Loop through all detected tags
                for (LLResultTypes.BarcodeResult fr : barcode) {
                    String barCodeData = fr.getData();

                    telemetry.addData("BarCode Data", barCodeData);
                    telemetry.update();
                }
            } else {
                telemetry.addLine("Valid result, but no Barcode detected.");
                telemetry.update();
            }
        } else {
            telemetry.addLine("No target detected.");
            telemetry.update();
        }
    }


    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        getFiducialResult(result, telemetry);

        getBarcodeResult(result, telemetry);

    }
}
