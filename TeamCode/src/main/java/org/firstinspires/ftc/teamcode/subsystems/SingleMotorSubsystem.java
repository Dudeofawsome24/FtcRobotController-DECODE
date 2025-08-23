package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class SingleMotorSubsystem extends SubsystemBase {

    private MotorEx motor;

    private Telemetry telemetry;

    private PIDFController controller;

    public SingleMotorSubsystem(final HardwareMap hMap, Telemetry telemetry, String name){

        this.telemetry = telemetry;

        //Get motors & servos from hardware map
        motor = new MotorEx(hMap, name);
    }

    public SingleMotorSubsystem(final HardwareMap hMap, Telemetry telemetry, String name, double kp, double ki, double kd, double kf){

        this.telemetry = telemetry;

        //Get motors & servos from hardware map
        motor = new MotorEx(hMap, name);

        controller = new PIDFController(kp, ki, kd, kf);
    }

    //Raw Power
    private void setPower(double power) {
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(power);
    }

    public Command setPowerCommand(Supplier<Double> power) {
        return new InstantCommand(() -> setPower(power.get()));
    }

    //Positional Control
    private void initPositionMotor(int position) {
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setTargetPosition(position);
        motor.set(0);
        motor.setPositionTolerance(10);
        controller.reset();
    }

    private void setPosition(int position) {
        double power = controller.calculate(motor.getCurrentPosition(), position);
        motor.set(power);
    }

    public Command setPositionCommand(Supplier<Integer> position) {
        return new FunctionalCommand(
            //Init
            () -> {
                initPositionMotor(position.get());
            },
            //Execute
            () -> {
                setPosition(position.get());
            },
            //End
            interrupted -> motor.stopMotor(),
            //Is finished
            () -> motor.atTargetPosition(),
            //requirements
            this
        );
    }

    //Velocity Control

}
