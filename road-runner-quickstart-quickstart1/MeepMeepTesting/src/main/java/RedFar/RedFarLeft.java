package RedFar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedFarLeft {
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
                                .lineToConstantHeading(new Vector2d(-47, -23))
                                .waitSeconds(0.5)
                                .addDisplacementMarker(40.1, () -> {

                                })
                                .lineToLinearHeading(new Pose2d(-58,-10, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(50,-25, Math.toRadians(0)))
                                .strafeRight(5)
                                .waitSeconds(1.0)
                                .strafeLeft(5)
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(-58,-10, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(0,0))
                                .lineToLinearHeading(new Pose2d(50,-25, Math.toRadians(0)))
                                .strafeRight(2)
                                .waitSeconds(1.0)
                                .strafeLeft(15)
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}