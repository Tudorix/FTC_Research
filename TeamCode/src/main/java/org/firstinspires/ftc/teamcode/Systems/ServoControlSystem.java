package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class ServoControlSystem {
    private static ServoControlSystem intakeSystem = null;
    
    public Servo Arm , Stopper , Plane;
    double ARM_IntakePos = 0.00, ARM_OuttakePos = 0.00;
    double STOP_IntakePos = 0.00, STOP_OuttakePos = 0.00;
    
    private boolean running = false;
    
    private ServoControlSystem(Robot instance){
        this.Arm = instance.rotateArm;
        this.Stopper = instance.stopper;
        this.ARM_IntakePos = instance.ROTATEARM_TAKE_POSITION;
        this.ARM_OuttakePos = instance.ROTATEARM_PLACE_POSITION;
        this.STOP_OuttakePos = instance.STOPPER_CLOSED;
        this.STOP_IntakePos = instance.STOPPER_OPEN;
        this.Plane = instance.plane;
    }
    
    //Stopper
    public void Stopper_Open(){
        Stopper.setPosition(STOP_IntakePos);
    }
    public void Stopper_Close(){
        Stopper.setPosition(STOP_OuttakePos);
    }
    
    //Arm
    public void Arm_Take(){
        Arm.setPosition(ARM_IntakePos);
    }
    public void Arm_Place(){
        Arm.setPosition(ARM_OuttakePos);
    }
    
    //Plane
    public void Launch_PLane(){
        Plane.setPosition(0.6);
    }
    
    public static synchronized ServoControlSystem getInstance(HardwareMap hardwareMap){
        if(intakeSystem == null){
            intakeSystem = new ServoControlSystem(Robot.getInstance(hardwareMap));
        }
        return intakeSystem;
    }
}
