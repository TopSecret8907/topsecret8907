package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "AutoSimple", group = "Adon")
public class AutoSimple extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;

    /* Define Poses - Converted from your PoseFactory logic */
    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose pose1 = new Pose(56, 20, Math.toRadians(180));
    private final Pose point2 = new Pose(33.4728, 81.6703, Math.toRadians(-16.6293));
    private final Pose point3 = new Pose(36.6558, 34.4783, Math.toRadians(93.8586));

    /* Define Paths */
    private Path path1, path2, path3;

    /** This method builds the paths using your coordinates */
    public void buildPaths() {
        // Path 1: Line from start to pose1
        path1 = new Path(new BezierLine(new Point(startPose), new Point(pose1)));
        path1.setLinearHeadingInterpolation(startPose.getHeading(), pose1.getHeading());

        // Path 2: Line from pose1 to point2 (Reversed)
//        path2 = new Path(new BezierLine(new Point(pose1), new Point(point2)));
//        path2.setLinearHeadingInterpolation(pose1.getHeading(), point2.getHeading());
//        path2.setReversed(true);

        // Path 3: Line from point2 to point3 (Reversed)
//        path3 = new Path(new BezierLine(new Point(point2), new Point(point3)));
//        path3.setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading());
//        path3.setReversed(true);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        // Simple State Machine to follow paths in order
        switch (pathState) {
            case 0: // Start Path 1
                follower.followPath(path1);
                pathState = 3;
                break;

            case 1: // Wait for Path 1 to finish
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    pathState = 2;
                }
                break;

            case 2: // Wait for Path 2 to finish
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    pathState = 3;
                }
                break;

            case 3: // Final State
                if (!follower.isBusy()) {
                    telemetry.addData("Status", "Done!");
                }
                break;
        }

        // Feedback
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        follower.telemetryDebug(telemetry);
    }
}