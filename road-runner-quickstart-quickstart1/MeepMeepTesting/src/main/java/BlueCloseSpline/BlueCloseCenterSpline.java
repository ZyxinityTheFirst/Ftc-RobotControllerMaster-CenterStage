package BlueCloseSpline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCloseCenterSpline {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);

        Pose2d blueClose = new Pose2d(12.5, 60, Math.toRadians(-90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(60), Math.toRadians(60), -20)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12.5, 60, Math.toRadians(-90)))
                                        .forward(42)
                                        .addTemporalMarker(() ->{

                                        })
                                        .waitSeconds(0.5)
                                        //*, Math.toRadians(-90)*/
                                        .lineToLinearHeading(new Pose2d(48,25,Math.toRadians(0)))
                                        .strafeLeft(10)
                                        .waitSeconds(0.5)
                                        .strafeRight(15)

                                        //Stack
                                        .lineToLinearHeading(new Pose2d(-60,-12, Math.toRadians(180)))
                                        .waitSeconds(0.5)
                                        .lineToLinearHeading(new Pose2d(48,20,Math.toRadians(0)))
                                        .strafeLeft(8)
                                        .waitSeconds(1.0)

                                        //UNCOMMENT FOR 4 PIXELS
//                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
//                                        .waitSeconds(0.5)
//                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))


                                        //Park :down_arrow:
                                        //Comment out if going for 4 pixels >:)
                                        .strafeLeft(30)
                                        .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}