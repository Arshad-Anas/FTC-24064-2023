package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public static double
            LEFT_PROP_LEFT_X = -46,
            LEFT_PROP_CENTER_X = -36,
            LEFT_PROP_RIGHT_X = -24,
            LEFT_PROP_LEFT_Y = -30,
            LEFT_PROP_CENTER_Y = -24,
            LEFT_PROP_RIGHT_Y = -30;

    public static boolean isRed = true;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double side = isRed ? 1 : -1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, -62, toRadians(90)))
                                .splineToSplineHeading(new Pose2d(0, 0, toRadians(90)), toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }
}