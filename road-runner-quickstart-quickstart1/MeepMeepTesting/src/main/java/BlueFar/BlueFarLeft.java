package BlueFar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFarLeft {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(60), Math.toRadians(60), -20)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-37,61, Math.toRadians(-90)))
                                        .splineTo(new Vector2d(-17, 35), Math.toRadians(0))
                                        .waitSeconds(1.0)
                                        .forward(6)
                                        .lineToConstantHeading(new Vector2d(-11,15))
                                        .splineToConstantHeading(new Vector2d(46,20), Math.toRadians(90))
                                        .strafeLeft(20)
                                        .waitSeconds(1.0)
                                        .strafeRight(20)
                                        .lineToLinearHeading(new Pose2d(-58,-11,Math.toRadians(180)))
                                        .waitSeconds(0.5)
                                        .lineToLinearHeading(new Pose2d(50,22, Math.toRadians(0)))
                                        .strafeLeft(7)
                                        .waitSeconds(1.0)
                                        .strafeRight(15)
                                        .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}