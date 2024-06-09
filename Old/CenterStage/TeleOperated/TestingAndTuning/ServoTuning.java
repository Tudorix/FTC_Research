package org.firstinspires.ftc.teamcode.CenterStage.TeleOperated.TestingAndTuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTuning extends OpMode {
    
    public Servo servo = null;
    public static String servoName = "";
    public static double Position= 0;
    
    @Override
    public void init(){
        this.servo = hardwareMap.get(Servo.class , servoName);
    }
    
    @Override
    public void loop() {
        servo.setPosition(Position);
    }
}

