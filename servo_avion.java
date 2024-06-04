package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.NanoClock;

public class ServoLauncher {
    public static void main(String[] args) {

        int initialPosition = 1; // Must be changed
        int launchPosition = 1000; // Must be changed

        try {
            while (true) {
                if (button.isPressed()) {
                    servo.setPosition(launchPosition);
                    NanoClock.clock().sleep(1000);// trebuie sters probabil

                    servo.setPosition(initialPosition);
                    break;
                }

                NanoClock.clock().sleep(100);// probabil nu trebuie
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            servo.setPosition(initialPosition);
        }
    }
}
