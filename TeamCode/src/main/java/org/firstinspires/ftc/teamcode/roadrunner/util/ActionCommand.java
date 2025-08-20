package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

public class ActionCommand implements Command {
    private final Action action;
    private boolean finished = false;
    private String debug;
    private Telemetry telemetry;

    public ActionCommand(Action action, String debug, Telemetry telemetry) {
        this.action = action;
        this.debug = debug;
        this.telemetry = telemetry;
        }

    @Override
    public Set<Subsystem> getRequirements() {
        return null;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if(AutoHelpers.autoDebug == true) {
            telemetry.addData("Action: ", debug);
            telemetry.update();
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}