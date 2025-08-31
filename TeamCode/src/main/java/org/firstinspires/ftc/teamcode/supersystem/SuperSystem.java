package org.firstinspires.ftc.teamcode.supersystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.DoubleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DoubleServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoSubsystem;

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


}
