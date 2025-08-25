package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class DoubleServoSubsystem extends SubsystemBase {

    private Servo servoLeft;
    private Servo servoRight;

    private Telemetry telemetry;

    public DoubleServoSubsystem(final HardwareMap hMap, Telemetry telemetry, String nameLeft, String nameRight){

        this.telemetry = telemetry;

        //Get servos from hardware map
        servoLeft = hMap.get(Servo.class, nameLeft);
        servoRight = hMap.get(Servo.class, nameRight);
    }

    private void setPosition(double[] position) {
        servoLeft.setPosition(position[0]);
        servoRight.setPosition(position[1]);
    }

    public Command setPositionCommand(Supplier<double[]> position) {
        return new InstantCommand(() -> setPosition(position.get()), this);
    }

}
