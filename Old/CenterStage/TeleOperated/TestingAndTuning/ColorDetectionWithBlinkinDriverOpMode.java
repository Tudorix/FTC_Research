package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated.TestingAndTuning;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

class ColorDetectionWithBlinkinDriverOpMode extends LinearOpMode {

    private RevBlinkinLedDriver blinkinLedDriver;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Aici trebuie schimbar dar nu sunt sigura cum
    private static final int WHITE_COLOR_VALUE = 0;
    private static final int GREEN_COLOR_VALUE = 1;
    private static final int PURPLE_COLOR_VALUE = 2;
    private static final int YELLOW_COLOR_VALUE = 3;

    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();

        while (opModeIsActive()) {
            detectAndSetLEDColor();
        }
    }

    private void initializeHardware() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor"); //acelasi senzor

    }

    private void detectAndSetLEDColor() {
        String detectedColor = detectColor();
        String setLEDColor = (detectedColor);
    }

    private boolean isWhite(int red, int green, int blue) {

        return (red > 200 && red < 255  && green > 200 && green < 255 && blue > 200 && blue<255);
    }

    private boolean isGreen(int red, int green, int blue) {

        return (red < 100 && green > 100 && green< 255 && blue < 100);
    }

    private boolean isPurple(int red, int green, int blue) {

        return (red > 100 && red < 255 && green < 100 && blue > 100 && blue < 255);
    }

    private boolean isYellow(int red, int green, int blue) {

        return (red > 150  && red < 255 && green > 150 && green < 255 && blue < 100);
    }

    private String detectColor() {
        int redValue = colorSensor.red();
        int greenValue = colorSensor.green();
        int blueValue = colorSensor.blue();


        if (isWhite(redValue, greenValue, blueValue)) {
            return "White";
        } else if (isGreen(redValue, greenValue, blueValue)) {
            return "Green";
        } else if (isPurple(redValue, greenValue, blueValue)) {
            return "Purple";
        } else if (isYellow(redValue, greenValue, blueValue)) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    private void setLEDColor(String detectedColor) {
        switch (detectedColor) {
            case "White":
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
                break;
            case "Green":
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            case "Purple":
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                break;
            case "Yellow":
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                break;
            default:
                setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                break;
        }
    }

    private void setBlinkinLedPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        // Set blinkin LED
        blinkinLedDriver.setPattern(pattern);
    }
}
