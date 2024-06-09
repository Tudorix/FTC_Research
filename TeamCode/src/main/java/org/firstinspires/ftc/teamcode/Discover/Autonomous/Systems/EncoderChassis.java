package org.firstinspires.ftc.teamcode.Discover.Autonomous.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Discover.HardwareClass;

public class EncoderChassis {
    
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    
    public EncoderChassis(HardwareClass hardwareClass){
        this.FL = null;
        this.FR = null;
        this.BR = null;
        this.BL = null;
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    
    public void Movement(int FRp,int FLp,int BRp , int BLp , double Power){
        //Reset encoders
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Set target position
        FL.setTargetPosition(FLp);
        FR.setTargetPosition(FRp);
        BL.setTargetPosition(BLp);
        BR.setTargetPosition(BRp);
        
        //Set motors to run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Set power to all motors
        FL.setPower(Power);
        FR.setPower(Power);
        BL.setPower(Power);
        BR.setPower(Power);
        
        while(FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()){
            //This waits until target position is reached
        }
        
        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
