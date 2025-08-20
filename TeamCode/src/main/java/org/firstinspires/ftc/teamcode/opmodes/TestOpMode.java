package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.supersystem.SuperSystem;

@TeleOp(name= "Test")
public class TestOpMode extends CommandOpMode {

    private SuperSystem s;

    private GamepadEx driver;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);

        s = new SuperSystem(hardwareMap, telemetry);

        //Controls
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            s.Intake()
        );
    }

}
