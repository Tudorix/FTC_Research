package org.firstinspires.ftc.teamcode.CenterStage.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

@Config
public class DistanceSystem {
    private static DistanceSystem single_instance = null;
    private boolean running = false;
    double treshold , tresholdPixel;
    DistanceSensor sensor = null;
    DistanceSensor pixel = null;
    public DistanceSystem(Robot instance){
        sensor = instance.stopperSensor;
        treshold = instance.minDistanceToBackboardCM;
    }
    
    /**
     * Get current distance form sensor
     */
    
    public double getDistance(){
        return sensor.getDistance(DistanceUnit.CM);
    }
    
    public boolean isOverDistance(){
        if(sensor.getDistance(DistanceUnit.CM) < treshold){
            return true;
        }
        return false;
    }
    
    public boolean isPixel(){
        if(pixel.getDistance(DistanceUnit.CM) < tresholdPixel){
            return true;
        }
        return false;
    }
    
   
    public static synchronized DistanceSystem getInstance(HardwareMap hMap){
        if(single_instance == null){
            single_instance = new DistanceSystem(Robot.getInstance(hMap));
        }
        return single_instance;
    }
}
