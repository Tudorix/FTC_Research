package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.systems.DriveTrain;
import org.firstinspires.ftc.teamcode.systems.GamepadInput;
import org.firstinspires.ftc.teamcode.systems.ServoControlSystem;
import org.firstinspires.ftc.teamcode.systems.SlidesSystem;

@TeleOp
@Config
public class TeleOp_01 extends OpMode {


    DriveTrain driveTrain = null;
    SlidesSystem slidesSystem = null;
    GamepadInput gamepadInput = null;


    @Override
    public void init(){
        driveTrain = DriveTrain.getInstance(hardwareMap,gamepad1);
        driveTrain.start();

        slidesSystem = SlidesSystem.getInstance(hardwareMap, gamepad2);
        slidesSystem.start();

        gamepadInput = GamepadInput.getInstance(gamepad1, gamepad2,  hardwareMap);
    }

    @Override
    public void loop(){
        gamepadInput.registerGamepad2Input();
    }
}
