package org.firstinspires.ftc.teamcode.customUtility;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadHelpers {

    public static Trigger getGamepadTrigger(GamepadEx gamepad, GamepadKeys.Trigger trigger, double threshold) {

        return new Trigger(() -> gamepad.getTrigger(trigger) > threshold);

    }

}
