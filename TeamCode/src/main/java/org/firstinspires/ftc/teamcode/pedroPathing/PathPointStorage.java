package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class PathPointStorage {
    private Follower follower;

    // Poses
    public final Pose startPose = new Pose(9, 111, Math.toRadians(-90));
    public final Pose scorePose = new Pose(16, 128, Math.toRadians(-45));
    public final Pose pickup1Pose = new Pose(30, 121, Math.toRadians(0));
    public final Pose pickup2Pose = new Pose(30, 131, Math.toRadians(0));
    public final Pose pickup3Pose = new Pose(45, 128, Math.toRadians(90));
    public final Pose parkPose = new Pose(68, 96, Math.toRadians(-90));

    // Paths
    public PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3;
    public PathChain scorePickup1, scorePickup2, scorePickup3, park;

    public PathPointStorage(Follower follower) {
        this.follower = follower;
        buildPaths();
    }

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(68, 110), // control point
                        parkPose
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }
}
