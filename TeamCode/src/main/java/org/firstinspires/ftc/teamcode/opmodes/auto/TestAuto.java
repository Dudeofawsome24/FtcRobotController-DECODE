package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.ArraySet;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.ActionCommand;
import org.firstinspires.ftc.teamcode.roadrunner.util.PoseStorage;
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@Config
@Autonomous(name = "Test Auto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    private static SuperSystem s;
    private static MecanumDrive drive;

    private static Pose2d initialPose;

    @Override
    public void runOpMode() throws InterruptedException {

        s = new SuperSystem(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, initialPose);

        initialPose = new Pose2d(0,0,Math.toRadians(0));

        TrajectoryActionBuilder dropOffPreload = drive.actionBuilder(initialPose)
            .lineToX(10)
            .lineToY(10)
            .endTrajectory();

        TrajectoryActionBuilder sample1Pickup = dropOffPreload.fresh()
            .lineToX(0)
            .lineToY(0)
            .endTrajectory();

        CommandScheduler.getInstance().schedule(
            new WaitUntilCommand(this::isStarted).andThen(

                new SequentialCommandGroup(
                    new ActionCommand(dropOffPreload.build()),
                    s.Intake(),
                    new ActionCommand(sample1Pickup.build()),

                    new InstantCommand(() -> {

                        PoseStorage.currentPose = new Pose2d(0,0,Math.toRadians(0));

                    })
                )

            )
        );

    }
}
