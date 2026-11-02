package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.List;
import pedroPathing.constants.BioBuzzFConstants;
import pedroPathing.constants.BioBuzzLConstants;

/** Java/Pedro replacement for the Blocks-based moveToPoint approach. */
@Autonomous(name = "BioBuzz Auto (route template)", group = "BioBuzz")
public class BioBuzzAuto extends OpMode {
    private static final double MAX_DRIVE_POWER = 0.3;
    private static final double SEGMENT_TIMEOUT_SECONDS = 8.0;
    // Placeholder origin: replace with the actual field starting pose.
    // All route coordinates are inches; all headings are radians (CCW positive).
    private static final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));

    private enum State { READY, FOLLOWING, COMPLETE, TIMED_OUT }

    private final List<Path> route = new ArrayList<>();
    private final ElapsedTime segmentTimer = new ElapsedTime();
    private Follower follower;
    private State state = State.READY;
    private int segmentIndex;

    /** Add destinations in order once the route is known. Empty means no movement. */
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
        return new Pose[] {
            // Example syntax only: new Pose(24, 0, Math.toRadians(0)),
            // Add actual field destinations here after verifying/tuning constants.
        };
    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap, BioBuzzFConstants.class, BioBuzzLConstants.class);
        follower.setMaxPower(MAX_DRIVE_POWER);
        follower.setStartingPose(START_POSE);
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
        telemetry.addData("Setup", "Verify offsets, encoder/motor directions; tune new robot");
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
    }
}
