package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PathPointStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.commands.FollowPathCommand;

@Autonomous
public class PedroAutoSample extends CommandOpMode {
    private Follower follower;
    private PathPointStorage paths;
    private HardwareMap hMap;

    // Mechanism commands - replace with actual subsystem commands
    private InstantCommand openOuttakeClaw() { return new InstantCommand(() -> {}); }
    private InstantCommand grabSample() { return new InstantCommand(() -> {}); }
    private InstantCommand scoreSample() { return new InstantCommand(() -> {}); }
    private InstantCommand level1Ascent() { return new InstantCommand(() -> {}); }

    @Override
    public void initialize() {
        // Initialize follower and paths
        follower = Constants.createFollower(hMap);
        follower.setStartingPose(new com.pedropathing.geometry.Pose(9, 111, Math.toRadians(-90)));
        paths = new PathPointStorage(follower);

        // Autonomous sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(follower, paths.scorePreload),
                openOuttakeClaw(),
                new WaitCommand(1000),

                new FollowPathCommand(follower, paths.grabPickup1).setGlobalMaxPower(0.5),
                grabSample(),
                new FollowPathCommand(follower, paths.scorePickup1),
                scoreSample(),

                new FollowPathCommand(follower, paths.grabPickup2),
                grabSample(),
                new FollowPathCommand(follower, paths.scorePickup2, 1.0),
                scoreSample(),

                new FollowPathCommand(follower, paths.grabPickup3),
                grabSample(),
                new FollowPathCommand(follower, paths.scorePickup3),
                scoreSample(),

                new FollowPathCommand(follower, paths.park, false),
                level1Ascent()
        );

        autonomousSequence.schedule();
    }

    @Override
    public void run() {

    }
}
