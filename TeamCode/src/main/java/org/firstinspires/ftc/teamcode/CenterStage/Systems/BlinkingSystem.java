package org.firstinspires.ftc.teamcode.CenterStage.Systems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Robot;

public class BlinkingSystem {

    public enum Colors{
        LIGHT_BLUE(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE),
        PURPLE(RevBlinkinLedDriver.BlinkinPattern.VIOLET),
        RED(RevBlinkinLedDriver.BlinkinPattern.RED),
        GREEN(RevBlinkinLedDriver.BlinkinPattern.GREEN),
        ORANGE(RevBlinkinLedDriver.BlinkinPattern.ORANGE),
        
        COLOR_PIXELS(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES);

        private RevBlinkinLedDriver.BlinkinPattern pattern;
        Colors(RevBlinkinLedDriver.BlinkinPattern pattern){
            this.pattern = pattern;
        }

        public RevBlinkinLedDriver.BlinkinPattern getPattern(){
            return this.pattern;
        }

    }
    private static BlinkingSystem instance = null;

    private RevBlinkinLedDriver ledDriver = null;

    private  BlinkingSystem(Robot robot){
        this.ledDriver = robot.blinkingDrive;
    }

    public void setColor(Colors color){
        ledDriver.setPattern(color.getPattern());
    }

    public static synchronized BlinkingSystem getInstance(Robot robot){
        if(instance == null) {
            instance = new BlinkingSystem(robot);
        }

        return instance;
    }
}
