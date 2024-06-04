package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class GamepadInput{

    static GamepadInput gamepadInput = null;
    Gamepad gp1, gp2;

    HardwareMap hwmap;

    Robot robot = null;
    DriveTrain driveTrain = null;
    SlidesSystem slidesSystem = null;
    ServoControlSystem servoControlSystem = null;
    IntakeSystem intakeSystem = null;
    


    /* states */

    boolean intakeState = false;

    private GamepadInput(Gamepad gp1, Gamepad gp2, HardwareMap hwmap){
        this.gp1 = gp1;
        this.gp2 = gp2;

        this.robot = Robot.getInstance(hwmap);

        this.intakeSystem = IntakeSystem.getInstance(robot);
        this.servoControlSystem = ServoControlSystem.getInstance(hwmap);


        servoControlSystem.Arm_Take();
        servoControlSystem.Stopper_Open();
    }

    public void registerGamepad2Input(){
        if(this.gp2.a) servoControlSystem.Stopper_Close();
        if(this.gp2.y) servoControlSystem.Stopper_Open();
        if(this.gp2.x) servoControlSystem.Arm_Take();
        if(this.gp2.b) servoControlSystem.Arm_Place();

        if(this.gp2.left_bumper && this.gp2.dpad_left) servoControlSystem.Launch_PLane();

        //if(this.gp2.dpad_right) servoControlSystem.Plus_Arm();
        //if(this.gp2.dpad_left) servoControlSystem.Minus_Arm();
        
        intakeSystem.setPowerIntake(gp2.right_stick_x);
    }

    public static synchronized GamepadInput getInstance(Gamepad gamepad1, Gamepad gamepad2, HardwareMap hwmap){
        if(gamepadInput == null)
            gamepadInput = new GamepadInput(gamepad1, gamepad2, hwmap);
        return gamepadInput;
    }
}
