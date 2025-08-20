package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d pose) {

        drive = new MecanumDrive(hardwareMap, pose);

    }

    private void drive(double leftX, double leftY, double rightX, double scale) {

        double heading = drive.localizer.getPose().heading.toDouble();

        Vector2d sticks = new Vector2d(
            leftY * scale,
            -leftX * scale
        );

        double rotX = sticks.x * Math.cos(-heading) - sticks.y * Math.sin(-heading);
        double rotY = sticks.x * Math.sin(-heading) + sticks.y * Math.cos(-heading);

        Vector2d updatedVector = new Vector2d(rotX, rotY);

        drive.setDrivePowers(new PoseVelocity2d(
            updatedVector,
            -rightX
        ));


        drive.updatePoseEstimate();

    }

    public Command driveCommand(Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightX, Supplier<Double> scale) {

        return new InstantCommand(() -> drive(leftX.get(), leftY.get(), rightX.get(), scale.get()), this);

    }

}
