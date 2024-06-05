package org.firstinspires.ftc.teamcode.Discover;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CenterStage.Robot;

public class HardwareClass {
    
    private static HardwareClass hardwareClass = null;
    public HardwareMap hardwareMap = null;
    
    /**
     *
     * Object declarations
     */
    
    
    private HardwareClass(HardwareMap hwMap) {
        /**{
         * Initialise comoponents
         */
        
    }
    
    public static synchronized HardwareClass getInstance(HardwareMap hwMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hwMap);
        return hardwareClass;
    }
}
