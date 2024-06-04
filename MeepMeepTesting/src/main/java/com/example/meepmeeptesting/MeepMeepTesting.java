package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                                            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                                            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                            .followTrajectorySequence(drive ->
                                                                              drive.trajectorySequenceBuilder(new Pose2d(-35.20, -60.98, Math.toRadians(90.00)))
                                                                                      .lineToLinearHeading(new Pose2d(-35.20, -35.76, Math.toRadians(0)))
                                                                                      .lineToConstantHeading(new Vector2d(-50.73, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(11.79, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -42))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(60.83, -11.87))
                                                                                      .build()
                                            );
        
        /*
         RED FAR AUTO
         
         drive.trajectorySequenceBuilder(new Pose2d(-35.20, -60.98, Math.toRadians(90.00)))
         
         -Right case:

                                                                                      .lineToLinearHeading(new Pose2d(-35.20, -35.76, Math.toRadians(0)))
                                                                                      .lineToConstantHeading(new Vector2d(-50.73, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(11.79, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -42))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(60.83, -11.87))
                                                                                              
         
         -Center case:
                                                                                      .lineToConstantHeading(new Vector2d(-35.20, -33.23))
                                                                                      .lineToLinearHeading(new Pose2d(-52.73, -46.25, Math.toRadians(0.00)))
                                                                                      .lineToConstantHeading(new Vector2d(-52.73, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(11.79, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -35.95))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(60.83, -11.87))
                                                                                                
         
         
         -Left case:
                                                                                      .lineToLinearHeading(new Pose2d(-35.20, -34.72, Math.toRadians(180.00)))
                                                                                      .lineToConstantHeading(new Vector2d(-32.49, -11.87))
                                                                                      .turn(Math.toRadians(180))
                                                                                      .lineToConstantHeading(new Vector2d(11.79, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -29.85))
                                                                                      .lineToConstantHeading(new Vector2d(48.83, -11.87))
                                                                                      .lineToConstantHeading(new Vector2d(60.83, -11.87))
        
                                                                                                
        
         */
        
        /*
         BLUE FAR AUT0
         
         drive.trajectorySequenceBuilder(new Pose2d(-35.20, 57.47, Math.toRadians(270)))
         
         Right case:
        
                                                                                      .lineToLinearHeading(new Pose2d(-34.20, 33.50, Math.toRadians(0)))
                                                                                      .lineToConstantHeading(new Vector2d(-50.73, 11.61))
                                                                                      .lineToConstantHeading(new Vector2d(20, 11.61))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 41))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 11.23))
                                                                                      .lineToConstantHeading(new Vector2d(60, 11.42))
         
         
         
         Center case:
                                                                                      .lineToConstantHeading(new Vector2d(-35.20, 34.25))
                                                                                      .lineToLinearHeading(new Pose2d(-52.73, 50.11, Math.toRadians(0)))
                                                                                      .lineToConstantHeading(new Vector2d(-52.73, 11.61))
                                                                                      .lineToConstantHeading(new Vector2d(20, 11.61))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 35.70))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 11.23))
                                                                                      .lineToConstantHeading(new Vector2d(60, 11.42))
         
         
         
         Left case:
         
                                                                                      .lineToLinearHeading(new Pose2d(-36.20, 33.50, Math.toRadians(180)))
                                                                                      .lineToLinearHeading(new Pose2d(-32.20, 11.61, Math.toRadians(0)))
                                                                                      .lineToConstantHeading(new Vector2d(20, 11.61))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 28.70))
                                                                                      .lineToConstantHeading(new Vector2d(48.98, 11.23))
                                                                                      .lineToConstantHeading(new Vector2d(60, 11.42))
         */
    
    
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}