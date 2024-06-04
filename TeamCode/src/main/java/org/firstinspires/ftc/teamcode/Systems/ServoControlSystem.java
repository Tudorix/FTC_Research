package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;


public class ServoControlSystem {
    private static ServoControlSystem intakeSystem = null;
    
    public Servo Arm, Stopper , Plane , Sweep;
    double ARM_IntakePosR = 0.94, ARM_OuttakePosR = 0.68;
    double STOP_Close = 0.38, STOP_OpenIN = 0.28, STOP_OpenOUT = 0.33;
    double SWEEP_Down = 0.6, SWEEP_Up = 0.95;
    
    private boolean running = false;
    
    private ServoControlSystem(Robot instance){
        this.Arm = instance.rotateArm;
        this.Stopper = instance.stopper;
        this.Sweep = instance.sweepMove;
        this.Plane = instance.plane;
    }
    
    /** Stopper */
    public void stopperOpen(){
        Stopper.setPosition(STOP_OpenIN);
    }
    public void stopperClose(){
        Stopper.setPosition(STOP_Close);
    }
    public void stopperOut(){ Stopper.setPosition(STOP_OpenOUT);}
    
    /** Sweeper */
    public void sweepDown(){
        Sweep.setPosition(SWEEP_Down);
    }
    public void sweepUp(){
        Sweep.setPosition(SWEEP_Up);
    }
    
    /** Arms */
    public void rotateArmTake(){
        Arm.setPosition(ARM_IntakePosR);
    }
    public void adjust(){
        Arm.setPosition(1);
    }
    public void goDown(){
        Arm.setPosition(0.92);
    }
    
    public void rotateArmPlace(){
        Arm.setPosition(ARM_OuttakePosR);
    }
    
    /** Plane */
    public void launchPLane(){
        Plane.setPosition(0.6);
    }
    
    public static synchronized ServoControlSystem getInstance(HardwareMap hardwareMap){
        if(intakeSystem == null){
            intakeSystem = new ServoControlSystem(Robot.getInstance(hardwareMap));
        }
        return intakeSystem;
    }
}
