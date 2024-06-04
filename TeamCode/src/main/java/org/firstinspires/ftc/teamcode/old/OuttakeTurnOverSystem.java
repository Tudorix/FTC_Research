package org.firstinspires.ftc.teamcode.old;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class OuttakeTurnOverSystem {
    
    DcMotorEx turnOverMotor = null;
    Gamepad gm2;
    
    private final int UPPOSITION = 0, DOWNPOSITION = 0;
    private static OuttakeTurnOverSystem single_instance = null;
    private boolean running = false;
    Thread OuttakeTurnOver = null;
    
    //PIDF
    public PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    
    public static int target = 0;
    
    private final double ticks_in_degree = 435 /180.0; // RPM to change
    
    public OuttakeTurnOverSystem(Robot instance, Gamepad gm2){
        //this.turnOverMotor = instance.turnOverMotor;
        this.gm2 = gm2;
    }
    
    public void start(){
        running = true;
        if(OuttakeTurnOver == null || !OuttakeTurnOver.isAlive()){
            OuttakeTurnOver = new Thread(() ->{
                while(running){
                    if(gm2.y) {
                        target = UPPOSITION;
                    }else if(gm2.a) {
                        target = DOWNPOSITION;
                    }else {
                        controller.setPID(p, i, d);
                        int motorPos = turnOverMotor.getCurrentPosition();
                        double pid = controller.calculate(motorPos, target);
                        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

                        double power = pid + ff;

                        turnOverMotor.setPower(power);

                        telemetry.addData("pos" , motorPos);
                        telemetry.addData("target " , target);
                        telemetry.update();
                    }
                }
            });
        }
        OuttakeTurnOver.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized OuttakeTurnOverSystem getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new OuttakeTurnOverSystem(Robot.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
