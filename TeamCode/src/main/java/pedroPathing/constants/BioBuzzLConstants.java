package pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/** Pinpoint settings copied from PinpointOdometryTest01, still awaiting measurement. */
public final class BioBuzzLConstants {
    private BioBuzzLConstants() {}

    static {
        // Explicit name replaces the Blocks helper's selection of the first Pinpoint.
        PinpointConstants.hardwareMapName = "pinpoint";
        // Pedro poses/paths remain IN INCHES; this unit applies to pod offsets only.
        PinpointConstants.distanceUnit = DistanceUnit.MM;
        // Pinpoint X offset = lateral location of forward pod; Y = forward location
        // of strafe pod. Both are signed distances from the robot's rotation center.
        PinpointConstants.forwardY = -143.95;
        PinpointConstants.strafeX = -90.1;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
    }
}
