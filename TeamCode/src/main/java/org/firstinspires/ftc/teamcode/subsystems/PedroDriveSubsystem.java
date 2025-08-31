package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import java.util.function.Supplier;

public class PedroDriveSubsystem extends SubsystemBase {

    private Follower follower;
    private Localizer localizer;
    public PedroDriveSubsystem(HardwareMap hMap) {
        follower = Constants.createFollower(hMap);
        localizer = new PinpointLocalizer(hMap, Constants.localizerConstants,new Pose(0, 0, 0));



    }

    public Localizer getLocalizer() {  // accessor for other classes
        return localizer;
    }

    private void drive(double forward, double lateral, double heading, boolean robotCentric) {
        follower.setTeleOpDrive(forward, lateral, heading, robotCentric);
    }

    public Command driveCommand(Supplier<Double> forward, Supplier<Double> lateral, Supplier<Double> heading, Supplier<Boolean> robotCentric) {

        return new RunCommand(() -> drive(forward.get(), lateral.get(), heading.get(), robotCentric.get()), this);

    }

}
