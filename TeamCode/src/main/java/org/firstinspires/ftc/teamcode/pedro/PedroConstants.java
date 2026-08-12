package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Pedro Pathing wiring: follower gains, Pinpoint geometry, and the factory that binds them to
 * the robot's swerve drive.
 * <p>
 * Everything here is in Pedro's units — <b>inches</b> and radians, field origin bottom-left,
 * heading 0 along +x, CCW positive. The rest of the codebase works in metres; the two never
 * meet because {@link PedroSwerveDrivetrain} only ever sees unitless power vectors.
 * <p>
 * All values below are starting points and must be tuned on the robot. See the bring-up order
 * in CLAUDE.md — none of these gains mean anything until localization is verified first.
 */
@Configurable
public class PedroConstants {

    /** Ceiling on follower output power. Start well below 1.0 while bringing auto up. */
    public static double MAX_POWER = 0.7;

    public static double NOMINAL_VOLTAGE = 12.0;
    public static boolean VOLTAGE_COMPENSATION = true;

    /**
     * Sign of the heading term in {@link PedroSwerveDrivetrain#calculateDrive}. Derived to be
     * +1 from Pedro's mecanum differential (a faster right side is a CCW rotation), but it is
     * the one part of the mapping not verifiable without the robot. Flip this from the
     * dashboard if the robot spins away from its target heading instead of onto it.
     */
    public static double HEADING_POWER_SIGN = 1.0;

    /** Robot mass in kilograms — used by the follower's braking model. Measure it. */
    public static double MASS_KG = 14.0;

    /**
     * Fraction of heading power applied while holding a point — which includes every
     * {@code turn}/{@code turnTo}, since those are implemented as "hold this pose with a new
     * heading". Pedro defaults this to 0.35 for a gentle hold; that is far too little authority
     * for this chassis, which needs most of its available power just to break static friction,
     * and shows up as a turn that starts fine then stalls and trembles as it closes in.
     * Lower it toward Pedro's default if the robot hunts around a held pose.
     */
    public static double HOLD_POINT_HEADING_SCALING = 1.0;

    /** Same idea for translation while holding. Pedro's default, kept as-is. */
    public static double HOLD_POINT_TRANSLATIONAL_SCALING = 0.45;

    /**
     * Heading error at which a turn is considered finished, degrees. Pedro's default is
     * 0.01 rad (0.57°) — tighter than this drivetrain can reliably settle, which would leave
     * {@code isTurning()} true forever and a turn that never reports complete.
     */
    public static double TURN_TOLERANCE_DEG = 2.0;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(MASS_KG)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.08, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.00005, 0.6, 0))
            .holdPointHeadingScaling(HOLD_POINT_HEADING_SCALING)
            .holdPointTranslationalScaling(HOLD_POINT_TRANSLATIONAL_SCALING)
            .turnHeadingErrorThreshold(Math.toRadians(TURN_TOLERANCE_DEG))
            .centripetalScaling(0.0005);

    /**
     * Pinpoint pod geometry. {@code forwardPodY} is the forward-facing pod's offset left of
     * robot centre; {@code strafePodX} is the strafe pod's offset forward of centre. Both in
     * inches, both signed — measure them on the robot, these are placeholders.
     */
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(2.36)
            .strafePodX(5.97)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = PathConstraints.defaultConstraints;

    /**
     * Builds a Follower bound to an existing {@link SwerveDrive}. The drive is passed in rather
     * than constructed here so the opmode can home the modules before the follower ever
     * commands them — an unhomed module refuses to move.
     */
    public static Follower createFollower(HardwareMap hardwareMap, SwerveDrive drive) {
        return createFollower(hardwareMap, new PedroSwerveDrivetrain(drive, hardwareMap));
    }

    /**
     * Overload for callers that want to keep a handle on the drivetrain — useful during
     * bring-up, where {@link PedroSwerveDrivetrain#debugString()} is the only view of the
     * vx/vy/omega the follower is actually commanding.
     */
    public static Follower createFollower(HardwareMap hardwareMap, PedroSwerveDrivetrain drivetrain) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .setDrivetrain(drivetrain)
                .pathConstraints(pathConstraints)
                .build();
    }
}
