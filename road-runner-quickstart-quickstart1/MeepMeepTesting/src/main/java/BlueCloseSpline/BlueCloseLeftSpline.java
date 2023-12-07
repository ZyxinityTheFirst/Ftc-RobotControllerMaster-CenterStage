package BlueCloseSpline;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueCloseLeftSpline {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800)
                .setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(0.8f);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(60), Math.toRadians(60), -20)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(12.5, 60, Math.toRadians(-90)))
                                        .splineTo(new Vector2d(29,30), Math.toRadians(0))
//                                        .lineToConstantHeading(new Vector2d(23,25))
                                        .addTemporalMarker(() ->{

                                        })
                                        .waitSeconds(0.5)

                                        //*, Math.toRadians(-90)*/
                                        .lineToConstantHeading(new Vector2d(48,40))
//                                        .lineToLinearHeading(new Pose2d(48,40,Math.toRadians(0)))
                                        .waitSeconds(0.5)
                                        .strafeRight(10)

                                        //Stack
                                        .splineToSplineHeading(new Pose2d(0,0, Math.toRadians(0)), Math.toRadians(180))
                                        .lineToLinearHeading(new Pose2d(-58,11, Math.toRadians(180)))
//                                        .lineToLinearHeading(new Pose2d(-58,-11, Math.toRadians(180)))
                                        .waitSeconds(1.0)
                                        .lineToLinearHeading(new Pose2d(48,10,Math.toRadians(0)))
                                        .strafeLeft(20)
                                        .waitSeconds(1.0)
                                        .strafeLeft(20)

                                        //UNCOMMENT FOR 4 PIXELS
//                                        .lineToLinearHeading(new Pose2d(-60,-11, Math.toRadians(180)))
//                                        .waitSeconds(0.5)
//                                        .lineToLinearHeading(new Pose2d(50,25,Math.toRadians(0)))


                                        //Park :down_arrow:
                                        //Comment out if going for 4 pixels >:)
                                        .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(myBot)
                .start();
    }
}