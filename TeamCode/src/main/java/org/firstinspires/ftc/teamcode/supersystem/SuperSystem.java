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

    //States
    private enum State {
        IDLE,
        HOLD_SAMPLE,
        HOLD_SPECIMEN,
        INTAKE_SAMPLE,
        INTAKE_SPECIMEN
    }

    private State currentState;

    public SuperSystem(HardwareMap hardwareMap, Telemetry telemetry) {

        //Single Motors
        intake = new SingleMotorSubsystem(hardwareMap, telemetry, "intakeMotor");

        //Single Servos
        claw = new SingleServoSubsystem(hardwareMap, telemetry, "claw");

        //Double Servos
        horizontal = new DoubleServoSubsystem(hardwareMap, telemetry, "leftH", "rightH");

        //Initialise Servos
        horizontal.setPositionCommand(() -> IntakeConstants.kFullStow);
        //claw.setPositionCommand(() -> TransferConstants.kClose);

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

    private void setState(State state) {
        currentState = state;
    }


}
