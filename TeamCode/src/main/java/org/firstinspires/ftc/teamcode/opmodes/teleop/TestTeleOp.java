package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.util.AutoHelpers;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@TeleOp(name = "Test", group = "TeleOp")
public class TestTeleOp extends CommandOpMode {

    //Max Speed
    private double maxSpeed = 1.0;

    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        SuperSystem s = new SuperSystem(hardwareMap, telemetry);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, AutoHelpers.poseStorage);

        drive.driveCommand(() -> driver.getLeftX(), () -> driver.getLeftY(), () -> driver.getRightX(), () -> maxSpeed);

        //Controls
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            s.Intake()
        );
    }

}
