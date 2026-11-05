package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.BioBuzzFConstants;
import pedroPathing.constants.BioBuzzLConstants;


/**
 * Java/Pedro replacement for the Blocks-based moveToPoint approach.
 */
@Autonomous(name = "BioBuzz Auto (route template)", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {
    private static final double MAX_DRIVE_POWER = 0.3;
    private static final double SEGMENT_TIMEOUT_SECONDS = 8.0;

    // Servo positions & timing constants
    public static double GATE_DOWN_POSITION = 0.0;
    public static double GATE_UP_POSITION = 1.0;
    public static double FLOWER_WEDGE_DEFAULT_POSITION = 0.0;
    public static double GATE_UP_DURATION_SECONDS = 1.5;

    // Default motor powers
    public static double INTAKE_POWER = 1.0;
    public static double FLYWHEEL_POWER = 0.85;
    public static double TRANSFER_POWER = 1.0;

    // Placeholder origin: replace with the actual field starting pose.
    // All route coordinates are inches; all headings are radians (CCW positive).
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));

    private enum State {READY, FOLLOWING, COMPLETE, TIMED_OUT}

    private final List<Path> route = new ArrayList<>();
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();

    private Follower follower;
    private State state = State.READY;
    private int segmentIndex;

    // Expansion Hub Motors
    private DcMotorEx intake;    // Port 0 (Expansion Hub)
    private DcMotorEx flywheel;  // Port 1 (Expansion Hub)
    private DcMotorEx transfer;  // Port 2 (Expansion Hub)

    // Control Hub Servos
    private Servo gate;          // Port 0 (Control Hub)
    private Servo flowerWedge;   // Port 1 (Control Hub)

    private boolean gateIsUp = false;


    /**
     * Add destinations in order once the route is known. Empty means no movement.
     */
    private void buildRoute() {
        route.clear();
        Pose previous = START_POSE;
        for (Pose destination : routeWaypoints()) {
            Path path = new Path(new BezierLine(new Point(previous), new Point(destination)));
            path.setLinearHeadingInterpolation(previous.getHeading(), destination.getHeading());
            route.add(path);
            previous = destination;
        }
    }

    private Pose[] routeWaypoints() {

        return new Pose[]{
                new Pose(56, 8, Math.toRadians(90)),
                new Pose(56, 36, Math.toRadians(180)),
                new Pose(76.5729, 90.586, Math.toRadians(-110.6509)),
                new Pose(53.966, 96.4272, Math.toRadians(-14.4871))
        };

    }


    @Override
    public void init() {
        follower = new Follower(hardwareMap, BioBuzzFConstants.class, BioBuzzLConstants.class);
        follower.setMaxPower(MAX_DRIVE_POWER);
        follower.setStartingPose(START_POSE);

        // HardwareMap setup for Expansion Hub motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HardwareMap setup for Control Hub servos
        gate = hardwareMap.get(Servo.class, "gate");
        flowerWedge = hardwareMap.get(Servo.class, "flowerWedge");

        // Initial servo positions
        setGateDown();
        flowerWedge.setPosition(FLOWER_WEDGE_DEFAULT_POSITION);

        buildRoute();
        reportTelemetry();
    }

    @Override
    public void init_loop() {
        // Update localization before START without commanding a path.
        follower.update();
        reportTelemetry();
    }

    @Override
    public void start() {
        segmentIndex = 0;
        if (route.isEmpty()) {
            finish(State.COMPLETE);
        } else {
            startSegment();
        }
    }

    private void startSegment() {
        follower.followPath(route.get(segmentIndex));
        segmentTimer.reset();
        state = State.FOLLOWING;
    }

    @Override
    public void loop() {
        follower.update(); // Required every loop, including while following a path.

        // Automatically reset the gate servo after the set duration when transfer is active
        if (gateIsUp && gateTimer.seconds() >= GATE_UP_DURATION_SECONDS) {
            setGateDown();
        }

        if (state == State.FOLLOWING) {
            if (segmentTimer.seconds() >= SEGMENT_TIMEOUT_SECONDS) {
                finish(State.TIMED_OUT);
            } else if (!follower.isBusy()) {
                segmentIndex++;
                if (segmentIndex < route.size()) {
                    startSegment();
                } else {
                    finish(State.COMPLETE);
                }
            }
        }
        reportTelemetry();
    }

    // --- Component Helper Methods ---

    public void startIntake(double power) {
        if (intake != null) intake.setPower(power);
    }

    public void stopIntake() {
        if (intake != null) intake.setPower(0);
    }

    public void startFlywheel(double power) {
        if (flywheel != null) flywheel.setPower(power);
    }

    public void stopFlywheel() {
        if (flywheel != null) flywheel.setPower(0);
    }

    /**
     * Starts transfer and raises the gate servo immediately.
     */
    public void startTransfer(double power) {
        if (transfer != null) transfer.setPower(power);
        if (power > 0) {
            setGateUp();
        } else {
            setGateDown();
        }
    }

    public void stopTransfer() {
        if (transfer != null) transfer.setPower(0);
        setGateDown();
    }

    public void setGateUp() {
        if (gate != null) gate.setPosition(GATE_UP_POSITION);
        gateTimer.reset();
        gateIsUp = true;
    }

    public void setGateDown() {
        if (gate != null) gate.setPosition(GATE_DOWN_POSITION);
        gateIsUp = false;
    }

    public void setFlowerWedgePosition(double position) {
        if (flowerWedge != null) flowerWedge.setPosition(position);
    }

    private void finish(State result) {
        follower.breakFollowing();
        state = result;
    }

    private void reportTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addData("State", state);
        telemetry.addData("Route segments", route.size());
        telemetry.addData("Segment index (zero based)", segmentIndex);
        telemetry.addData("X (in)", pose.getX());
        telemetry.addData("Y (in)", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHeading()));

        // Hardware Telemetry
        telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0);
        telemetry.addData("Flywheel Power", flywheel != null ? flywheel.getPower() : 0);
        telemetry.addData("Transfer Power", transfer != null ? transfer.getPower() : 0);
        telemetry.addData("Gate Pos", gate != null ? gate.getPosition() : 0);
        telemetry.addData("Gate Is Up", gateIsUp);
        telemetry.addData("FlowerWedge Pos", flowerWedge != null ? flowerWedge.getPosition() : 0);

        if (route.isEmpty()) {
            telemetry.addData("Route", "No waypoints configured; robot will stay still");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        stopIntake();
        stopFlywheel();
        stopTransfer();
        setGateDown();
    }
}
