package org.firstinspires.ftc.teamcode.CenterStage.Auxiliary;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class PIDConstants {
    public static double Kp = 0.2;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0;   // Derivative gain
}