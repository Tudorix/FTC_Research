package org.firstinspires.ftc.teamcode.CenterStage.Autonomous.DetectionPreload;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousConstants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;
import org.firstinspires.ftc.teamcode.Vision.TSESystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous
@Config
public class PreloadBlue extends LinearOpMode {

    Robot robot = null;
    SampleMecanumDrive drive = null;
    TSESystem tseDetection = null;
    SweeperSystem sweeperSystem = null;

    TrajectorySequence frontTraj, rightTraj, midTraj, leftTraj = null;


    /* Constants */

    public static double sidesTseDistance = 0;
    
    public static double displacePixel = 20;

    int detectionCase = 0;
    HardwareMap hardwareMap = null;

    public static double fi = AutonomousConstants.tileSize*1.1;

    @Override
    public void runOpMode() throws InterruptedException {


            robot = Robot.getInstance(hardwareMap);

            drive = new SampleMecanumDrive(hardwareMap);

            tseDetection = TSESystem.getInstance(hardwareMap);
    
            sweeperSystem = SweeperSystem.getInstance(hardwareMap);

            tseDetection.start("blue");
            while(!isStarted()) {
                detectionCase = tseDetection.SeeCase();
                telemetry.addData("Case", detectionCase);
                telemetry.update();
            }
            waitForStart();


            if(isStopRequested()) {return;}

            frontTraj = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .forward(fi)
                    .build();

            rightTraj = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .turn(Math.toRadians(-45))
                                .forward(displacePixel)
                                .back(displacePixel)
                    .turn(Math.toRadians(90))
                    .build();

            leftTraj  = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                    .turn(Math.toRadians(45))
                                .forward(displacePixel)
                                .back(displacePixel)
                    .turn(Math.toRadians(-45))
                    .build();

            midTraj  = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                               .forward(displacePixel)
                               .back(displacePixel)
                    .build();

            drive.followTrajectorySequence(frontTraj);


            switch (detectionCase){
                case 1:
                    drive.followTrajectorySequence(leftTraj);
                    break;
                case 2:
                    drive.followTrajectorySequence(midTraj);
                    break;
                case  3:
                    drive.followTrajectorySequence(rightTraj);
                    break;
                default:
                    drive.followTrajectorySequence(midTraj);
                    break;

            }
            
            


    }
}
