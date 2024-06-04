package org.firstinspires.ftc.teamcode.CenterStage.Systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

@Config
public class SweeperSystem {
    
    public DcMotor sweeper = null;
    public double MaxPower;
    private static SweeperSystem sweeperSystem = null;
    private boolean running = false;
    
    private SweeperSystem(Robot instance){
        this.sweeper = instance.SweeperMotor;
        this.MaxPower = instance.MAX_POWER_SWEEP;
    }
    
    /**
     * Give custom power to motor
     */
    
    public void setSweeperPower(double Power){
        sweeper.setPower(Power);
    }
    
    /**
     * Give set power to motor
     */
    
    public void startSweeper(){
        sweeper.setPower(-1 * MaxPower);
    }
    public void stopSweeper(){
        sweeper.setPower(0);
    }
    public void startReverseSweeper(){
        sweeper.setPower(-1 * MaxPower);
    }
    
    public static synchronized SweeperSystem getInstance(HardwareMap hardwareMap){
        if(sweeperSystem == null){
            sweeperSystem = new SweeperSystem(Robot.getInstance(hardwareMap));
        }
        return sweeperSystem;
    }
}
