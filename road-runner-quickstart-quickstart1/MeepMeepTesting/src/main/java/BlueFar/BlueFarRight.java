package BlueFar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFarRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 30, Math.toRadians(60), Math.toRadians(60), -20)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-37,61, Math.toRadians(-90)))
                                        .lineToConstantHeading(new Vector2d(-46,23))
                                        .waitSeconds(0)
                                        .addDisplacementMarker(38,() -> {

                                        })
                                        .waitSeconds(0.5)
                                        //*, Math.toRadians(-90)*/
                                        .splineToConstantHeading(new Vector2d(-5,0),Math.toRadians(0))
                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
                                        .strafeLeft(15)
                                        .waitSeconds(1.0)
                                        .strafeRight(15)
                                        //Stack
                                .lineToLinearHeading(new Pose2d(-60,-10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
                                        .strafeLeft(2)
                                        .waitSeconds(1.0)
                                        .strafeRight(2)
                                        //Park :down_arrow:
                                .strafeRight(15)
                                .forward(5)
                                        .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}