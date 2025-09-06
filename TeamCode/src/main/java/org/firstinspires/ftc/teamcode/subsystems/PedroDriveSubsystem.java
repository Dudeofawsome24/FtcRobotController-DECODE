package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;

public class PedroDriveSubsystem extends SubsystemBase {

    private Follower follower;
    private VisionSubsystem vision;

    private PIDFController rotationController;
    private PIDFController positionController;

    public PedroDriveSubsystem(HardwareMap hMap, Follower follower, VisionSubsystem vision, double[] rotationPIDF, double[] positionPIDF) {

        this.follower = follower;
        this.vision = vision;

        rotationController = new PIDFController(
            rotationPIDF[0],
            rotationPIDF[1],
            rotationPIDF[2],
            rotationPIDF[3]
        );

        positionController = new PIDFController(
            positionPIDF[0],
            positionPIDF[1],
            positionPIDF[2],
            positionPIDF[3]
        );

    }

    public PedroDriveSubsystem(HardwareMap hMap, Follower follower) {
        this.follower = follower;
        follower.setStartingPose(new Pose(0,0, Math.toRadians(0)));
    }

    //drive
    private void drive(double leftY, double leftX, double rightX, boolean isRobotCentric) {
        follower.update();
        follower.setTeleOpDrive(-leftX, leftY, -rightX, isRobotCentric);
    }

    public Command Drive(Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> isRobotCentric) {

        return new RunCommand(

            () -> drive(leftY.get(), leftX.get(), rightX.get(), isRobotCentric.get()),
                this

        );

    }

    //Rotation align to tag
    private void alignRotationTag(double targetRotation) {
        double error = rotationController.calculate(vision.getAprilTagPose().getOrientation().getYaw(), targetRotation);
        drive(0,0,error, true);
    }

    private void resetRotationController() {
        if (rotationController != null) rotationController.reset();
    }

    public Command AlignRotationTag(Supplier<Double> targetRotation) {
        return new FunctionalCommand(
            //Init
            () -> {
                resetRotationController();
            },
            //Execute
            () -> {
                alignRotationTag(targetRotation.get());
            },
            //End
            interrupted -> drive(0,0,0,false),
            //Is finished
            () -> Math.abs(vision.getAprilTagPose().getOrientation().getYaw() - targetRotation.get()) <= 5,
            //requirements
            this
        );
    }

    //Position align to tag
    private void alignPositionTag(double targetX, double targetZ) {
        double errorX = positionController.calculate(vision.getAprilTagPose().getPosition().x, targetX);
        double errorZ = positionController.calculate(vision.getAprilTagPose().getPosition().z, targetZ);

        drive(errorZ,errorX,0, true);
    }

    private void resetPositionController() {
        if (rotationController != null) rotationController.reset();
    }

    public Command AlignPositionTag(Supplier<Double> targetX, Supplier<Double> targetZ) {
        return new FunctionalCommand(
            //Init
            () -> {
                resetPositionController();
            },
            //Execute
            () -> {
                alignPositionTag(targetX.get(), targetZ.get());
            },
            //End
            interrupted -> drive(0,0,0,false),
            //Is finished
            () -> (Math.abs(vision.getAprilTagPose().getPosition().x - targetX.get()) <= 5) && (Math.abs(vision.getAprilTagPose().getPosition().z - targetZ.get()) <= 5),
            //requirements
            this
        );
    }

}
