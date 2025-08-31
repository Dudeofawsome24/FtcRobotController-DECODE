package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.util.function.Supplier;

public class PedroDriveSubsystem extends SubsystemBase {

    private Follower follower;

    public PedroDriveSubsystem(HardwareMap hMap) {

        Localizer localizer = new PinpointLocalizer(hMap);

        follower = new Follower(hMap, localizer, FConstants.class, LConstants.class);

    }

    private void drive(double forward, double lateral, double heading, boolean robotCentric) {
        follower.setTeleOpMovementVectors(forward, lateral, heading, robotCentric);
    }

    public Command driveCommand(Supplier<Double> forward, Supplier<Double> lateral, Supplier<Double> heading, Supplier<Boolean> robotCentric) {

        return new RunCommand(() -> drive(forward.get(), lateral.get(), heading.get(), robotCentric.get()), this);

    }

}
