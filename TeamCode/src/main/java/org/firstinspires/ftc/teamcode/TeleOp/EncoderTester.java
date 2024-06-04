package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class EncoderTester extends OpMode {
    Robot robot = null;

    @Override
    public void init() {
        robot = Robot.getInstance(hardwareMap);
        robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addLine("BL: " + String.valueOf(robot.BL.getCurrentPosition()));
        telemetry.addLine("BR: " + String.valueOf(robot.BR.getCurrentPosition()));
        telemetry.addLine("FL: " + String.valueOf(robot.FL.getCurrentPosition()));
        telemetry.addLine("FR: " + String.valueOf(robot.FR.getCurrentPosition()));
    }
}
