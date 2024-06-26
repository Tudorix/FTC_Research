package org.firstinspires.ftc.teamcode.CenterStage.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Systems.SweeperSystem;
import org.firstinspires.ftc.teamcode.Vision.AprilTagSystem;
import org.firstinspires.ftc.teamcode.Vision.TSESystem;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

public class RedDetectAndPlace extends LinearOpMode {
    Robot robot = null;
    SampleMecanumDrive drive = null;
    TSESystem tseDetection = null;
    
    AprilTagSystem aprilDetection = null;
    SweeperSystem sweeperSystem = null;
    
    TrajectorySequence frontTraj, rightTraj, midTraj, leftTraj = null;
    
    TrajectorySequence goToBoardTraj = null;
    
    
    /* Constants */
    
    public static double sidesTseDistance = 0;
    
    int detectionCase = 0;
    
    public static double fi = AutonomousConstants.tileSize*1.1;
    
    public static double boardRight = AutonomousConstants.tileSize * 1.2;
    
    public static double frontBoard = AutonomousConstants.tileSize / 2;
    
    
    
    private int getZone(){
        aprilDetection = AprilTagSystem.getInstance(hardwareMap);
        
        aprilDetection.start();
        int aprilCase = aprilDetection.SeeTag();
        
        return aprilCase;
    }
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        
        robot = Robot.getInstance(hardwareMap);
        
        drive = new SampleMecanumDrive(hardwareMap);
        
        tseDetection = TSESystem.getInstance(hardwareMap);
    
        sweeperSystem = SweeperSystem.getInstance(hardwareMap);
        
        tseDetection.start("red");
        while(!isStarted()) {
            detectionCase = tseDetection.SeeCase();
            telemetry.addData("Case", detectionCase);
            telemetry.update();
        }
        waitForStart();
        
        tseDetection.stop();
        
        if(isStopRequested()) {return;}
        
        frontTraj = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                            .forward(fi)
                            .build();
        
        rightTraj = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                            .turn(Math.toRadians(90))
                            .UNSTABLE_addTemporalMarkerOffset(2,()->{
                                sweeperSystem.setSweeperPower(-0.4);
                            })
                            .UNSTABLE_addTemporalMarkerOffset(4,()->{
                                sweeperSystem.stopSweeper();
                            })
                            .waitSeconds(4)
                            .turn(Math.toRadians(-90))
                            .build();
        
        leftTraj  = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                            .turn(Math.toRadians(-90))
                            .addTemporalMarker(2,()->{
                                sweeperSystem.setSweeperPower(-0.4);
                            })
                            .addTemporalMarker(4,()->{
                                sweeperSystem.stopSweeper();
                            })
                            .waitSeconds(4)
                            .turn(Math.toRadians(90))
                            .build();
        
        midTraj  = drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                           .addTemporalMarker(0,()->{
                               sweeperSystem.setSweeperPower(-0.4);
                           })
                           .addTemporalMarker(2,()->{
                               sweeperSystem.stopSweeper();
                           })
                           .waitSeconds(2)
                           .build();
   
        goToBoardTraj = drive.trajectorySequenceBuilder(midTraj.end())
                                 .back(fi/2)
                                 .strafeRight(boardRight)
                                 .forward(frontBoard)
                                 .turn(Math.toRadians(-90))
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
        
        drive.followTrajectorySequence(goToBoardTraj);
        
        int detectedCase = getZone();
        
        switch (detectedCase){
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }
        
        
        
    }
}
