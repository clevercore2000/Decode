package org.firstinspires.ftc.teamcode.Opmodes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.AutoConstants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.pedro.PedroConstants;

/**
 * Localization-only bring-up. Reads the Pinpoint pose and drives <b>nothing</b> — push the
 * robot around by hand and check the numbers.
 * <p>
 * This is the gate before any powered path following. Confirm that:
 * <ul>
 *   <li>pushing the robot forward increases x, pushing it left increases y (Pedro's frame is
 *       +x forward, +y left, heading 0 along +x, CCW positive);</li>
 *   <li>rotating counter-clockwise increases heading;</li>
 *   <li>returning the robot to where it started reads back roughly the starting pose — a large
 *       drift here means pod offsets, encoder resolution or encoder directions are wrong in
 *       {@link PedroConstants#localizerConstants}.</li>
 * </ul>
 * Modules are not homed and no follower path is run, so this is safe to run on the floor.
 */
@TeleOp(name = "Pinpoint Localization Test", group = "Tuning")
public class PinpointLocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwerveDrive drive = new SwerveDrive(hardwareMap);
        Follower follower = PedroConstants.createFollower(hardwareMap, drive);
        follower.setStartingPose(AutoConstants.START);

        telemetry.addLine("Pinpoint Localization Test");
        telemetry.addLine("Nothing will be driven. Press START, then push the robot by hand.");
        telemetry.update();

        waitForStart();

        Pose start = AutoConstants.START;

        while (opModeIsActive()) {
            // updatePose() only refreshes localization; update() would also command the
            // drivetrain, which is exactly what this opmode must not do.
            follower.updatePose();
            Pose pose = follower.getPose();

            telemetry.addData("X (in)", "%.2f", pose.getX());
            telemetry.addData("Y (in)", "%.2f", pose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
            telemetry.addLine();
            telemetry.addData("Δ from start", "%.2f, %.2f, %.1f°",
                    pose.getX() - start.getX(),
                    pose.getY() - start.getY(),
                    Math.toDegrees(pose.getHeading() - start.getHeading()));
            telemetry.addLine();
            telemetry.addLine("Push forward -> X up | push left -> Y up | CCW -> heading up");
            telemetry.update();
        }
    }
}
