package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.Supplier;

public class SingleMotorSubsystem extends SubsystemBase {

    private DcMotor motor;

    private Telemetry telemetry;

    public SingleMotorSubsystem(final HardwareMap hMap, Telemetry telemetry, String name){

        this.telemetry = telemetry;

        //Get motors & servos from hardware map
        motor = hMap.get(DcMotor.class, name);
    }

    private void setPower(double power) {
        motor.setPower(power);
    }

    public Command setPowerCommand(Supplier<Double> power) {
        return new InstantCommand(() -> setPower(power.get()), this);
    }

}
