package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class DoubleMotorSubsystem extends SubsystemBase {

    private MotorEx motorLeader;
    private MotorEx motorFollower;

    private MotorGroup motorGroup;

    private Telemetry telemetry;

    private PIDFController controller;

    public DoubleMotorSubsystem(final HardwareMap hMap, Telemetry telemetry, String nameLeader, String nameFollower){

        this.telemetry = telemetry;

        //Get motors from hardware map
        motorLeader = new MotorEx(hMap, nameLeader);
        motorLeader.setRunMode(Motor.RunMode.RawPower);

        motorFollower = new MotorEx(hMap, nameFollower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);

        motorGroup = new MotorGroup(motorLeader, motorFollower);
    }

    public DoubleMotorSubsystem(final HardwareMap hMap, Telemetry telemetry, String nameLeader, String nameFollower, double kp, double ki, double kd, double kf){

        this.telemetry = telemetry;

        //Get motors & servos from hardware map
        motorLeader = new MotorEx(hMap, nameLeader);
        motorLeader.setRunMode(Motor.RunMode.RawPower);

        motorFollower = new MotorEx(hMap, nameFollower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);

        motorGroup = new MotorGroup(motorLeader, motorFollower);

        controller = new PIDFController(kp, ki, kd, kf);
    }

    //Raw Power
    private void setPower(double power) {
        motorGroup.set(power);
    }

    public Command setPowerCommand(Supplier<Double> power) {
        return new InstantCommand(() -> setPower(power.get()));
    }

    //Positional Control
    private void setPosition(int position) {
        double output = controller.calculate(motorGroup.getCurrentPosition(), position);
        motorGroup.set(output);
    }

    public Command setPositionCommand(Supplier<Integer> position) {
        return new FunctionalCommand(
            //Init
            () -> {
                resetController();
            },
            //Execute
            () -> {
                setPosition(position.get());
            },
            //End
            interrupted -> motorGroup.stopMotor(),
            //Is finished
            () -> Math.abs(motorGroup.getCurrentPosition() - position.get()) <= 10,
            //requirements
            this
        );
    }

    //Velocity Control
    private void runVelocity(double velocity) {
        double output = controller.calculate(motorGroup.getVelocity(), velocity);
        motorGroup.set(output);
    }

    public Command setVelocityCommand(Supplier<Double> velocity) {
        return new FunctionalCommand(
            () -> {
                resetController();
            },
            () -> {
                runVelocity(velocity.get());
            },
            interrupted -> motorGroup.stopMotor(),
            () -> false,
            this
        );
    }

    //Shared Functions
    private void resetController() {
        if (controller != null) controller.reset();
    }


}
