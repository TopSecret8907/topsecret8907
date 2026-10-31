package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.FConstantsfilfil;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutoSimple", group = "Adon")
public class AutoSimple extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    private static final double PAUSE_SECONDS = 1.0;

    // Recorded endpoints with intermediate poses at 1/3 and 2/3 of the route.
    // All headings are in radians, interpolated between the recorded headings.
    private final Pose startPose = new Pose(-247.3367718073327, 108.07706577571359, 5.924096584320068);
    private final Pose pose1 = new Pose(-255.00129439386484, 142.14533608103676, 5.65025011698405);
    private final Pose point2 = new Pose(-262.665816980397, 176.2136063863599, 5.37640364964803);
    private final Pose point3 = new Pose(-270.3303395669291, 210.28187669168307, 5.102557182312012);

    /* Define Paths */
    private Path path1, path2, path3;

    /** This method builds the paths using your coordinates */
    public void buildPaths() {
        // Path 1: Line from start to pose1
        path1 = new Path(new BezierLine(new Point(startPose), new Point(pose1)));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading());

        // Path 2: Line from pose1 to point2
        path2 = new Path(new BezierLine(new Point(pose1), new Point(point2)));
        path2.setLinearHeadingInterpolation(pose1.getHeading(), point2.getHeading());

        // Path 3: Line from point2 to the recorded endpoint
        path3 = new Path(new BezierLine(new Point(point2), new Point(point3)));
        path3.setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading());
    }

    @Override
    public void init() {
        // Send all telemetry to both the Driver Station and FTC Dashboard.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();


    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        requestOpModeStop();
    }

    @Override
    public void loop() {
        follower.update();

        // Simple State Machine to follow paths in order
        switch (pathState) {
            case 0: // Start Path 1
                follower.followPath(path1);
                pathState = 1;
                break;

            case 1: // Wait for Path 1 to finish
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    pathTimer.resetTimer();
                    pathState = 5;
                }
                break;

            case 2: // Wait for Path 2 to finish
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    pathTimer.resetTimer();
                    pathState = 6;
                }
                break;

            case 5: // Pause at the first intermediate point
                telemetry.addData("Status", "Paused at point 1");
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_SECONDS) {
                    follower.followPath(path2);
                    pathState = 2;
                }
                break;

            case 6: // Pause at the second intermediate point
                telemetry.addData("Status", "Paused at point 2");
                if (pathTimer.getElapsedTimeSeconds() >= PAUSE_SECONDS) {
                    follower.followPath(path3);
                    pathState = 4;
                }
                break;

            case 4: // Wait for Path 3 before entering the stop state
                if (!follower.isBusy()) {
                    pathState = 3;
                }
                break;

            case 3: // Final State
                stop();
                telemetry.addData("Status", "Stopped");
                break;
        }

        // Feedback
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        follower.telemetryDebug(telemetry);
        telemetry.update();
    }
}
