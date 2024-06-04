package org.firstinspires.ftc.teamcode.CenterStage.Autonomous.Parkings;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
@Config

public class ParkingRight extends LinearOpMode {

    SampleMecanumDrive drive = null;
    Trajectory parkRightTrajectory = null;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        parkRightTrajectory =  drive.trajectoryBuilder(new Pose2d(AutonomousConstants.tileSize/0, 0, Math.toRadians(0)))
                .strafeRight(AutonomousConstants.parkRightDistance)
                .build();


        waitForStart();
        if(isStopRequested()) {return;}

        drive.followTrajectory(parkRightTrajectory);



    }
}
