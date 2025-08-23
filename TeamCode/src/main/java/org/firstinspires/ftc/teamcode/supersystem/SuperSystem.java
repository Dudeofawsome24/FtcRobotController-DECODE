package org.firstinspires.ftc.teamcode.supersystem;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.subsystems.DoubleServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SingleServoSubsystem;

public class SuperSystem extends SubsystemBase {

    //Single motors
    private static SingleMotorSubsystem intake;

    //Single servos
    private static SingleServoSubsystem claw;

    //Double servos
    private static DoubleServoSubsystem horizontal;

    private static DoubleServoSubsystem arm;

    //States
    private enum State {
        IDLE,
        SAMPLE_HOLD
    }

    private State currentState;

    public SuperSystem(HardwareMap hardwareMap, Telemetry telemetry) {

        //Single Motors
        intake = new SingleMotorSubsystem(hardwareMap, telemetry, "intake");

        //Single Servos
        claw = new SingleServoSubsystem(hardwareMap, telemetry, "claw");

        //Double Servos
        horizontal = new DoubleServoSubsystem(hardwareMap, telemetry, "leftH", "rightH");
        arm = new DoubleServoSubsystem(hardwareMap, telemetry, "leftA", "rightA");
        //Initialise Servos
        horizontal.setPositionCommand(() -> IntakeConstants.kFullStow);
        claw.setPositionCommand(() -> TransferConstants.kClose);

        //Set state
        setState(State.IDLE);
    }

    public Command Intake() {
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

    private void setState(State state) {
        currentState = state;
    }


}
