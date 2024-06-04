package org.firstinspires.ftc.teamcode.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTuning extends OpMode {
    
    public Servo rotateArm = null;
    public static String servoName = "ARM";
    public static double ROTATEARM = 0;
    
    @Override
    public void init(){
        
        this.rotateArm = hardwareMap.get(Servo.class , servoName);
    }
    
    @Override
    public void loop() {
        rotateArm.setPosition(ROTATEARM);
    }
}

