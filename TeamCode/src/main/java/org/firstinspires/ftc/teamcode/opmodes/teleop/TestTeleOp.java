package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@TeleOp(name = "Test", group = "TeleOp")
public class TestTeleOp extends CommandOpMode {

    private static SuperSystem s;
    private static DriveSubsystem drive;

    private GamepadEx driver;

    //Max Speed
    private double maxSpeed = 1.0;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);

        s = new SuperSystem(hardwareMap, telemetry);
        drive = new DriveSubsystem(hardwareMap, PoseStorage.currentPose);

        drive.driveCommand(() -> driver.getLeftX(), () -> driver.getLeftY(), () -> driver.getRightX(), () -> maxSpeed);

        //Controls
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            s.Intake()
        );
    }

}
