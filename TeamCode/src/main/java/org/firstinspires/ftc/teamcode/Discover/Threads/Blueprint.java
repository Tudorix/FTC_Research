package org.firstinspires.ftc.teamcode.Discover.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Discover.HardwareClass;

public class Blueprint {
    private static Blueprint single_instance = null;
    Thread Blueprint = null;
    private boolean running = false;
    
    /**
     *
     * Object declarations
     */

    Gamepad gm;
    
    public Blueprint(HardwareClass instance, Gamepad gm){
        this.gm = gm;
        /**{
         * Initialise comoponents
         */
    }
    
    public void start(){
        running = true;
        if(Blueprint == null || !Blueprint.isAlive()){
            Blueprint = new Thread(() ->{
                while(running){
                    /**{
                     * This is the code that will be executed in the thread
                     */
                }
            });
        }
        Blueprint.start();
    }
    
    public void stop(){
        running = false;
    }
    
    public boolean getStatus(){
        return running;
    }
    
    public static synchronized Blueprint getInstance(HardwareMap hMap, Gamepad gm){
        if(single_instance == null){
            single_instance = new Blueprint(HardwareClass.getInstance(hMap), gm);
        }
        return single_instance;
    }
}
