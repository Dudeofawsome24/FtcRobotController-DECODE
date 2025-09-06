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
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@Autonomous
public class PedroAutoSample extends CommandOpMode {
    private PathPointStorage paths;

    private SuperSystem s = new SuperSystem(hardwareMap, telemetry);

    // Mechanism commands - replace with actual subsystem commands
    /*private InstantCommand openOuttakeClaw() { return new InstantCommand(() -> {}); }
    private InstantCommand grabSample() { return new InstantCommand(() -> {}); }
    private InstantCommand scoreSample() { return new InstantCommand(() -> {}); }
    private InstantCommand level1Ascent() { return new InstantCommand(() -> {}); }*/

    @Override
    public void initialize() {
        // Initialize follower and paths
        s.getFollower().setStartingPose(paths.startPoseLine1);
        paths = new PathPointStorage(s.getFollower());

        // Autonomous sequence
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
                new FollowPathCommand(s.getFollower(), paths.testLine1),
                new WaitCommand(1000),
                new FollowPathCommand(s.getFollower(), paths.testLine2)
        );

                /* FollowPathCommand(follower, paths.grabPickup1).setGlobalMaxPower(0.5),
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
                level1Ascent()*/

        waitForStart();
        autonomousSequence.schedule();
    }

    @Override
    public void run() {

    }
}
