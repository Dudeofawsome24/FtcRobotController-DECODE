package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.customUtility.GamepadHelpers;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@TeleOp(name = "PedroTeleop", group = "TeleOp")
public class PedroTeleOp extends CommandOpMode {

    @Override
    public void initialize() {
        GamepadEx driver = new GamepadEx(gamepad1);
        TriggerReader driverLeftTrigger = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader driverRightTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);

        GamepadEx operator = new GamepadEx(gamepad2);
        TriggerReader operatorLeftTrigger = new TriggerReader(operator, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader operatorRightTrigger = new TriggerReader(operator, GamepadKeys.Trigger.RIGHT_TRIGGER);

        SuperSystem s = new SuperSystem(hardwareMap, telemetry);

        //Controls
        s.getFollower().startTeleopDrive();
        s.Drive(() -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> true).schedule();

        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            s.Intake()
        ).whenReleased(
            s.IntakeOff()
        );

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            s.ClawClose()
        ).whenReleased(
            s.ClawOpen()
        );

        GamepadHelpers.getGamepadTrigger(driver, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.5).whileActiveContinuous(
            s.HorizontalExtend()
        );

        GamepadHelpers.getGamepadTrigger(driver, GamepadKeys.Trigger.LEFT_TRIGGER, 0.5).whileActiveContinuous(
            s.HorizontalStow()
        );

        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            s.ScoreSpec()
        ).whenReleased(
            s.StowArm()
        );
    }

}
