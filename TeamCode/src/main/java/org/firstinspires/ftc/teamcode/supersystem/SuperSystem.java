package org.firstinspires.ftc.teamcode.supersystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class SuperSystem extends SubsystemBase {

    //Single motors
    private static SingleMotorSubsystem intake;

    //Double motors
    private static DoubleMotorSubsystem lift;

    //Single servos
    private static SingleServoSubsystem claw;

    //Double servos
    private static DoubleServoSubsystem horizontal;
    private static DoubleServoSubsystem arm;

    //Drive
    private static Follower follower;

    //Vision
    private static VisionSubsystem vison;

    //States
    private enum State {
        IDLE,
        HOLD_SAMPLE,
        HOLD_SPECIMEN,
        INTAKE_SAMPLE,
        INTAKE_SPECIMEN
    }

    private State currentState;

    public SuperSystem(HardwareMap hMap, Telemetry telemetry) {

        //Single Motors
        intake = new SingleMotorSubsystem(hMap, telemetry, "intakeMotor");

        //Double Motors
        lift = new DoubleMotorSubsystem(hMap, telemetry, "topLift", "bottomLift", 0,0,0,0);

        //Single Servos
        claw = new SingleServoSubsystem(hMap, telemetry, "claw");

        //Double Servos
        horizontal = new DoubleServoSubsystem(hMap, telemetry, "leftH", "rightH");
        arm = new DoubleServoSubsystem(hMap, telemetry, "leftA", "rightA");

        //Drive
        follower = Constants.createFollower(hMap);

        //Vision
        vison = new VisionSubsystem(hMap, telemetry);

        //Initialise Servos
        initServos();

        //Set state
        setState(State.IDLE);
    }

    public Command Intake() {
        return intake.setPowerCommand(() -> IntakeConstants.kIntakeOn);
    }

    public Command IntakeOff() {
        return intake.setPowerCommand(() -> IntakeConstants.kIntakeIdle);
    }

    public Command ClawClose() {
        return claw.setPositionCommand(() -> TransferConstants.kClose);
    }

    public Command ClawOpen() {
        return claw.setPositionCommand(() -> TransferConstants.kOpen);
    }

    public Command HorizontalStow() {
        return horizontal.setPositionCommand(() -> IntakeConstants.kFullStow);
    }

    public Command HorizontalExtend() {
        return horizontal.setPositionCommand(() -> IntakeConstants.kFullExtend);
    }

    public Command ScoreSpec() {
        return new SequentialCommandGroup(
                claw.setPositionCommand(() -> TransferConstants.kClose),
                arm.setPositionCommand(() -> TransferConstants.kArmScore)
        );
    }

    public Command StowArm() {
        return new SequentialCommandGroup(
            claw.setPositionCommand(() -> TransferConstants.kOpen),
            arm.setPositionCommand(() -> TransferConstants.kArmStow)
        );


    }

    //Super System helpers
    private void setState(State state) {
        currentState = state;
    }

    private void initServos() {
        horizontal.setPositionCommand(() -> IntakeConstants.kFullStow);
        claw.setPositionCommand(() -> TransferConstants.kClose);
    }

    /*public Command Intake() {
        return new ConditionalCommand(
            //Retracts and transfers if sample in intake
            new SequentialCommandGroup(
                intake.setPowerCommand(() -> IntakeConstants.kIntakeIdle),
                horizontal.setPositionCommand(() -> IntakeConstants.kFullStow),
                claw.setPositionCommand(() -> TransferConstants.kClose)
            ),
            //otherwise keep intake on
            new SequentialCommandGroup(
                horizontal.setPositionCommand(() -> IntakeConstants.kFullExtend),
                intake.setPowerCommand(() -> IntakeConstants.kIntakeOn)
            ),
            () -> currentState == State.SAMPLE_HOLD
        );
    }*/

    //drive
    private void drive(double leftY, double leftX, double rightX, boolean isRobotCentric) {
        follower.setTeleOpDrive(-leftY, -leftX, -rightX, isRobotCentric);
    }

    public Command Drive(Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> isRobotCentric) {

        return new RunCommand(

            () -> drive(leftY.get(), leftX.get(), rightX.get(), isRobotCentric.get())

        );

    }

    private void alignRotationTag(double targetRotation) {
        double kp = 0.5;

        double error = (targetRotation - vison.getAprilTagPose().getOrientation().getYaw()) * kp;

        drive(0,0,error, true);
    }

    public Command AlignRotationTag(Supplier<Double> targetRotation) {
        return new InstantCommand(() -> alignRotationTag(targetRotation.get()));
    }

    private void alignPositionTag(double targetX, double targetZ) {
        double kp = 0.5;

        double errorX = (targetX - vison.getAprilTagPose().getPosition().x) * kp;
        double errorZ = (targetX - vison.getAprilTagPose().getPosition().z) * kp;

        drive(errorZ,errorX,0, true);
    }

    public Command AlignPositionTag(Supplier<Double> targetX, Supplier<Double> targetZ) {
        return new InstantCommand(() -> alignPositionTag(targetX.get(), targetZ.get()));
    }

    public Follower getFollower() {
        return follower;
    }

}
