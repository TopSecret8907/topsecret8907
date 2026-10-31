package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PinpointOdometryTest01 (Blocks to Java)")
public class PinpointOdometryTest01 extends LinearOpMode {

  private DcMotor frontLeft;
  private DcMotor backLeft;
  private DcMotor frontRight;
  private DcMotor backRight;

  double currentr;
  double currentx;
  double currenty;

  @Override
  public void runOpMode() {
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");

    waitForStart();
    if (opModeIsActive()) {
      // setup pinpoint odometry
      PinpointBlocks.encoderResolution(PinpointBlocks.FourBarOdometryPod(DistanceUnit.MM), DistanceUnit.MM);
      // need better measurement
      PinpointBlocks.offsets(DistanceUnit.MM, 145, -100);
      // need to check what values go here
      PinpointBlocks.reverseEncoders(false, false);
      while (opModeIsActive()) {
        PinpointBlocks.update();
        //moveToPoint(null, null, null);
        updateTelemetry2();
      }
    }
  }

  private void moveToPoint(double targetx, double targety, double targetr) {
    double er;
    double ex;
    double ey;
    double rotation;
    double xpower;
    double ypower;
    telemetry.addData("targetr", Double.parseDouble(JavaUtil.formatNumber(targetr, 3)));
    telemetry.addData("targetx", Double.parseDouble(JavaUtil.formatNumber(targetx, 3)));
    telemetry.addData("targety", Double.parseDouble(JavaUtil.formatNumber(targety, 3)));
    currentr = PinpointBlocks.orientation(AngleUnit.DEGREES);
    currentx = PinpointBlocks.xPosition(DistanceUnit.MM);
    currenty = PinpointBlocks.yPosition(DistanceUnit.MM);
    er = targetr - currentr;
    ex = targetx - currentx;
    ey = targety - currenty;
    if (Math.abs(er) > 180) {
      rotation = -(Math.abs(er) / 360);
    } else {
      rotation = Math.abs(er) / 360;
    }
    xpower = ex / Math.max(Math.abs(ex), Math.abs(ey));
    ypower = ey / Math.max(Math.abs(ex), Math.abs(ey));
    frontLeft.setPower(-(ypower + xpower + rotation));
    backLeft.setPower((ypower - xpower) + rotation);
    frontRight.setPower((ypower - xpower) - rotation);
    backRight.setPower(-((ypower + xpower) - rotation));
  }

  private void updateTelemetry2() {
    double targetx;
    double targety;
    double targetr;

    currentr = PinpointBlocks.orientation(AngleUnit.DEGREES);
    currentx = PinpointBlocks.xPosition(DistanceUnit.MM);
    currenty = PinpointBlocks.yPosition(DistanceUnit.MM);
    telemetry.addData("currentr", Double.parseDouble(JavaUtil.formatNumber(currentr, 3)));
    telemetry.addData("currentx", Double.parseDouble(JavaUtil.formatNumber(currentx, 3)));
    telemetry.addData("currenty", Double.parseDouble(JavaUtil.formatNumber(currenty, 3)));
    telemetry.update();
  }
}
