package pedroPathing;

import static pedroPathing.Config.ConfigFile.CLOSE_ERROR_THRESHOLD;
import static pedroPathing.Config.ConfigFile.CONFIGkD;
import static pedroPathing.Config.ConfigFile.CONFIGkDClose;
import static pedroPathing.Config.ConfigFile.CONFIGkI;
import static pedroPathing.Config.ConfigFile.CONFIGkIClose;
import static pedroPathing.Config.ConfigFile.CONFIGkP;
import static pedroPathing.Config.ConfigFile.CONFIGkPClose;
import static pedroPathing.Config.ConfigFile.DEADZONE;
import static pedroPathing.Config.ConfigFile.REV_CLOSE_THRESHOLD;
import static pedroPathing.Config.ConfigFile.REVkD;
import static pedroPathing.Config.ConfigFile.REVkDClose;
import static pedroPathing.Config.ConfigFile.REVkP;
import static pedroPathing.Config.ConfigFile.REVkPClose;
import static pedroPathing.Config.ConfigFile.SERVO_POS_DOWN;
import static pedroPathing.Config.ConfigFile.SERVO_POS_MID;
import static pedroPathing.Config.ConfigFile.SERVO_POS_UP;
import static pedroPathing.Config.ConfigFile.TICK_STEP;
import static pedroPathing.Config.ConfigFile.loopTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import pedroPathing.constants.FConstantsfilfil;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TeleopFilfil", group = "adon")
public class TeleopFilfil extends OpMode {

    private Follower follower;
//    private DcMotorEx launcher;
//    private DcMotorEx launcher2;
    private DcMotorEx intakeMotor;


    private ElapsedTime pidTimer = new ElapsedTime();

    private double kP, kI, kD;
    private double launcherError;
    private double integralSum = 0;
    private double lastError = 0;
    private double pidOutput = 0;
    private boolean launcherOn = false;
    private double targetVelocity = 0;
    private double PidInputSpeed = 0.68;
    private double targetTicks = 0;

    private boolean prevRightBumper = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevLeftStickButton = false;
    private boolean prevRightStickButton = false;
    private boolean intakeOn = false;
    private boolean prevIntakeButton = false;

    private Limelight3A limelight;


    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstantsfilfil.class, LConstants.class);
        follower.setStartingPose(new Pose(0, 0, 0));
        pidTimer.reset();

//        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
//        launcher2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        launcher2.setDirection(DcMotorSimple.Direction.FORWARD);



        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);





        // Get the Limelight from the config
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(4);
//
//        // Start streaming data
//        limelight.start();

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        launcherOn = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double runPIDFHybrid(double target, double current, double currentPower) {
        launcherError = target - current;

        if (Math.abs(launcherError) > DEADZONE) {
            integralSum += launcherError * loopTime;
        }

        double derivative = (launcherError - lastError) / loopTime;
        double pidCorrection = (kP * launcherError) + (kI * integralSum) + (kD * derivative);
        lastError = launcherError;

        return currentPower + pidCorrection;
    }



    @Override
    public void loop() {
//        double currentVelocity = launcher.getVelocity();

//        if (Math.abs(targetVelocity - currentVelocity) > CLOSE_ERROR_THRESHOLD) {
//            kP = CONFIGkP;
//            kI = CONFIGkI;
//            kD = CONFIGkD;
//        } else {
//            kP = CONFIGkPClose;
//            kI = CONFIGkIClose;
//            kD = CONFIGkDClose;
//        }
//
//        if (gamepad1.right_bumper && !prevRightBumper) {
//            launcherOn = !launcherOn;
//        }

        if (gamepad1.b && !prevIntakeButton) {
            intakeOn = !intakeOn;
            intakeMotor.setPower(intakeOn ? 1.0 : 0.0);

        }

        if (gamepad1.left_stick_button && !prevLeftStickButton) {
            PidInputSpeed -= 0.01;
        }

        if (gamepad1.right_stick_button && !prevRightStickButton) {
            PidInputSpeed += 0.01;
        }

        PidInputSpeed = clamp(PidInputSpeed, 0.0, 1.0);





//        if (launcherOn) {
//            targetVelocity = (Math.round((PidInputSpeed * 1960) / 20.0) * 20) - 10;
//            if (pidTimer.seconds() >= loopTime) {
//                pidOutput = runPIDFHybrid(targetVelocity, currentVelocity, launcher.getPower());
//                launcher.setPower(pidOutput);
//                launcher2.setPower(pidOutput);
//                pidTimer.reset();
//            }
//        } else {
//            launcher.setPower(0);
//            launcher2.setPower(0);
//            targetVelocity = 0;
//            pidOutput = 0;
//            integralSum = 0;
//            lastError = 0;
//            launcherError = 0;
//        }
        if (!follower.isBusy()) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.5, true);
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Grab AprilTag results
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Loop through all detected tags
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    int tagId = fr.getFiducialId();

                    telemetry.addData("Tag ID", tagId);
                }
            } else {
                telemetry.addLine("Valid result, but no AprilTags detected.");
            }
        } else {
            telemetry.addLine("No target detected.");
        }



//        telemetry.addData("Launcher On", launcherOn);
//        telemetry.addData("Launcher On", launcherOn);
//        telemetry.addData("Launcher Power", launcher.getPower());
//        telemetry.addData("Launcher Target", targetVelocity);
//        telemetry.addData("Launcher Speed", currentVelocity);
//        telemetry.addData("Launcher Input %", PidInputSpeed);
//        telemetry.addData("Launcher Error", launcherError);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        prevRightBumper = gamepad1.right_bumper;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevLeftStickButton = gamepad1.left_stick_button;
        prevRightStickButton = gamepad1.right_stick_button;
        prevIntakeButton = gamepad1.b;

        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
    }
}
