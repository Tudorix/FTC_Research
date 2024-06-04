package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadRumble{
    public static void wrongRumble(Gamepad gp){
        gp.runRumbleEffect(
            new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 3.0, 100)
                .addStep(0.0, 0.0, 100)
                .addStep(3.0, 0.0, 100)
                .build()
        );
    }
    public static void okRumble(Gamepad gp){
        gp.runRumbleEffect(
                new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, 1.0, 200)
                        .addStep(1.0, 0.0, 200)
                        .build()
        );
    }

}
