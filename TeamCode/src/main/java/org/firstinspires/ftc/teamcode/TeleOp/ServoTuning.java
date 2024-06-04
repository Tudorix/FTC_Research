package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.systems.ServoControlSystem;

@TeleOp
@Config
public class ServoTuning extends OpMode {

    Robot robot = null;
    ServoControlSystem servoControlSystem = null;

    public static double TAKE , PLACE , OPEN , CLOSE;

    @Override
    public void init(){
        robot = Robot.getInstance(hardwareMap);
        servoControlSystem = ServoControlSystem.getInstance(hardwareMap);

        servoControlSystem.Arm_Take();
        servoControlSystem.Stopper_Open();
    }

    @Override
    public void loop(){
        if(gamepad2.left_bumper) servoControlSystem.Launch_PLane();

        if(gamepad2.a) servoControlSystem.Stopper.setPosition(CLOSE);
        if(gamepad2.y) servoControlSystem.Stopper.setPosition(OPEN);
    
        if(gamepad2.x) servoControlSystem.Arm.setPosition(TAKE);
        if(gamepad2.b) servoControlSystem.Arm.setPosition(PLACE);
        

    }
}
