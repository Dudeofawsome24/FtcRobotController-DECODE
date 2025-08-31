package org.firstinspires.ftc.teamcode.pedroPathing.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;

public class HoldPointCommand extends CommandBase {
    private final Follower follower;
    private final Pose pose;
    private final boolean isFieldCentric;

    public HoldPointCommand(Follower follower, Pose pose, boolean isFieldCentric) {
        this.follower = follower;
        this.pose = pose;
        this.isFieldCentric = isFieldCentric;
    }

    @Override
    public void initialize() {
        if (!isFieldCentric) {
            pose.plus(follower.getPose());
        }

        follower.holdPoint(pose);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        follower.resumePathFollowing();
    }
}
