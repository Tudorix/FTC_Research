package org.firstinspires.ftc.teamcode.TeleOperated.TestingAndTuning;

import com.qualcomm.robotcore.hardware.Gamepad;
public class gamepad_rumble {

    private Gamepad gamepad;
    private long rumbleStartTime=0;
    private boolean isRumbling = false;
    private final long RUMBLE_DURATION = 5000; // 5 seconds

    public gamepad_rumble (Gamepad gamepad) {
        this.gamepad = gamepad;
    }
/*
    public void update() {
        boolean isConditionMet = getState();

        if (isConditionMet && !isRumbling) {
            startRumble();
        } else if (!isConditionMet && isRumbling) {
            stopRumble();
        }

        if (isRumbling && rumbleTimer.milliseconds() > RUMBLE_DURATION) {
            stopRumble();
        }
    }*/

    private void startRumble() {
        gamepad.runRumbleEffect(
                new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, 3.0, 100)
                        .addStep(0.0, 0.0, 100)
                        .addStep(3.0, 0.0, 100)
                        .build()
        );

        isRumbling = true;
    }

    private void stopRumble() {
        gamepad.runRumbleEffect(
                new Gamepad.RumbleEffect.Builder()
                        .addStep(0.0, 1.0, 200)
                        .addStep(1.0, 0.0, 200)
                        .build()
        );

        isRumbling = false;
    }

    private boolean getState() {
        //Condition
        return gamepad.left_stick_y > 0.5;
    }
}
