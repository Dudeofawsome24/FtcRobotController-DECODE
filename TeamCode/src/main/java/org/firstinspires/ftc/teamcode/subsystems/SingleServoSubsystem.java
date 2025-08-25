package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class SingleServoSubsystem extends SubsystemBase {

    private Servo servo;

    private Telemetry telemetry;

    public SingleServoSubsystem(final HardwareMap hMap, Telemetry telemetry, String name){

        this.telemetry = telemetry;

        //Get servo from hardware map
        servo = hMap.get(Servo.class, name);
    }

    private void setPosition(double position) {
        servo.setPosition(position);
    }

    public Command setPositionCommand(Supplier<Double> position) {
        return new InstantCommand(() -> setPosition(position.get()), this);
    }

}
