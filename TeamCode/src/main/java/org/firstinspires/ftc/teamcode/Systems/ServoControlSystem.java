package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class ServoControlSystem {
    public Servo Arm , Stopper , Plane;
    private static ServoControlSystem intakeSystem = null;

    double ARM_IntakePos = 0.00, ARM_OuttakePos = 0.00;
    double STOP_IntakePos = 0.00, STOP_OuttakePos = 0.00;
    private boolean running = false;
    Thread IntakeSystem = null;

    private ServoControlSystem(Robot instance){
        this.Arm = instance.rotateArm;
        this.Stopper = instance.stopper;
        this.ARM_IntakePos = instance.ROTATEARM_TAKE_POSITION;
        this.ARM_OuttakePos = instance.ROTATEARM_PLACE_POSITION;
        this.STOP_OuttakePos = instance.STOPPER_CLOSED;
        this.STOP_IntakePos = instance.STOPPER_OPEN;
        this.Plane = instance.plane;
        Plane.setPosition(0);
    }
    
    public void Stopper_Open(){
        Stopper.setPosition(STOP_IntakePos);
    }
    public void Stopper_Close(){
        Stopper.setPosition(STOP_OuttakePos);
    }
    public void Arm_Take(){
        Arm.setPosition(ARM_IntakePos);
    }
    public void Arm_Place(){
        Arm.setPosition(ARM_OuttakePos);
    }
    public void Launch_PLane(){
        Plane.setPosition(0.8);
    }
    
    public void Plus_Arm(){
        ARM_IntakePos -= 0.05;
        ARM_OuttakePos -= 0.05;
        Arm_Take();
    }
    
    public void Minus_Arm(){
        ARM_IntakePos += 0.05;
        ARM_OuttakePos += 0.05;
        Arm_Place();
    }
    
    public static synchronized ServoControlSystem getInstance(HardwareMap hardwareMap){
        if(intakeSystem == null){
            intakeSystem = new ServoControlSystem(Robot.getInstance(hardwareMap));
        }
        return intakeSystem;
    }
}
