package RedFar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFarRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);

        Pose2d pose2d = new Pose2d(-42,-60, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(51.5662), Math.toRadians(51.5662), -20)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.5,-62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-36.5,-33.5, Math.toRadians(-180)))
                                .back(6.5)
                                .forward(1)
                                .lineToConstantHeading(new Vector2d(-58,-12))
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(50,-12, Math.toRadians(0)))
                                .strafeRight(30)
                                .waitSeconds(1.0)
                                .strafeLeft(30)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(-58,-12, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(50,-12, Math.toRadians(0)))
//                                .strafeLeft(15)
//                                .splineToLinearHeading(new Pose2d(-20,-34), Math.toRadians(0))
//                                .lineToConstantHeading(new Vector2d(10,-34))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}