package BlueClose;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCloseLeft {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(51.5662), Math.toRadians(51.5662), -20)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12.5, 60, Math.toRadians(-90)))
                                        .lineToConstantHeading(new Vector2d(23,25))
                                        .waitSeconds(0.5)
                                        .addDisplacementMarker(35,() -> {

                                        })
                                        .waitSeconds(0.5)
                                        //*, Math.toRadians(-90)*/
                                        .lineToLinearHeading(new Pose2d(50,40,Math.toRadians(0)))
                                        .waitSeconds(0.5)
                                        .strafeRight(25)

                                        //Stack
                                        .lineToLinearHeading(new Pose2d(-58,-11, Math.toRadians(180)))
                                        .waitSeconds(0.5)
                                        .lineToLinearHeading(new Pose2d(50,15,Math.toRadians(0)))
                                        .waitSeconds(0.5)
                                        .strafeLeft(25)
                                        .waitSeconds(0.5)
                                        .strafeRight(17)

                                        //UNCOMMENT FOR 4 PIXELS
//                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
//                                        .waitSeconds(0.5)
//                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))


                                        //Park :down_arrow:
                                        //Comment out if going for 4 pixels >:)
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