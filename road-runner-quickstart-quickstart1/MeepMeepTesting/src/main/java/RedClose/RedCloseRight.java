package RedClose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedCloseRight {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);

        Pose2d redClose = new Pose2d(12.5, -60, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(51.5662), Math.toRadians(51.5662), -20)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.5,-61, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(23,-23))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(37.05, () -> {

                                })
                                .lineToLinearHeading(new Pose2d(50,-18, Math.toRadians(0)))
                                .strafeRight(25)
                                .waitSeconds(1.0)
                                .strafeLeft(25)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(-58,-10, Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(48,-18, Math.toRadians(0)))
                                .strafeRight(10)
                                .waitSeconds(1.0)
                                .strafeLeft(15)
                                .turn(Math.toRadians(130))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}