package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.BlinkingSystem;

//@TeleOp
//@Config
public class LedOpMode extends OpMode {
    
    public static BlinkingSystem.Colors col = BlinkingSystem.Colors.ORANGE;
    Robot robot = null;
    BlinkingSystem bs = null;
    @Override
    public void init() {
        this.robot = Robot.getInstance(hardwareMap);
        this.bs = BlinkingSystem.getInstance(robot);
    }

    @Override
    public void loop() {
        this.bs.setColor(col);
    }
}
