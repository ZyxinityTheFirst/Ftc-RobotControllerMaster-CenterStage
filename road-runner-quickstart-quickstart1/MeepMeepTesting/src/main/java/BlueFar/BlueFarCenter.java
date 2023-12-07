package BlueFar;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFarCenter {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 40, Math.toRadians(51.5662), Math.toRadians(51.5662), -20)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-37,61, Math.toRadians(-90)))
                                .forward(42)
                                .waitSeconds(0.5)
                                .addDisplacementMarker(42,() -> {

                                })
                                .lineToLinearHeading(new Pose2d(-56,11, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(-5,0))
                                .lineToLinearHeading(new Pose2d(50,22,Math.toRadians(0)))
                                .strafeLeft(15)
                                .waitSeconds(0.5)
                                .strafeRight(15)
//-67, 12
                                .lineToLinearHeading(new Pose2d(-58,-11,Math.toRadians(180)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(50,22, Math.toRadians(0)))
                                .strafeLeft(5)
                                .waitSeconds(0.5)
                                .strafeRight(5)

//                                .lineToLinearHeading(new Pose2d(-60,-10, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))
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