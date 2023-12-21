package Colours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "ColourSubMatProgram")
public class ColourSubMat extends LinearOpMode {
    ColourSubMat.ColourSubMatPipeline pipeline;
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException {


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new ColourSubMat.ColourSubMatPipeline();
        webcam.setPipeline(pipeline);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        while (!isStopRequested() && opModeInInit()){
            telemetry.addData("Location: ", pipeline.getLocation());
            telemetry.addData("Left Side: ", pipeline.getLeftCenter());
            telemetry.addData("Center: ", pipeline.getCenterColor());
            telemetry.addData("Threshold: ",  ColourSubMat.ColourSubMatPipeline.threshold);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()){
            sleep(50);
        }

    }

    @Config
    public static class ColourSubMatPipeline extends OpenCvPipeline{


        private final AtomicReference<Mat> lastFrame = new AtomicReference<>(new Mat());

        private Location.Side location = Location.Side.RIGHT;
        public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

        private Rect leftZoneArea;
        private Rect centerZoneArea;

        private Mat finalMat = new Mat();

        public static int blueLeftX = (int)(800 * (1.0 / 3.0)); // adjusted
        public static int blueLeftY = (int)(367 * (4.0 / 9.0)); // adjusted

        public static int blueCenterX = (int)(1175 * (1.0 / 3.0)); // adjusted
        public static int blueCenterY = (int)(117 * (4.0 / 9.0)); // adjusted

        public static int redLeftX = SquareConstant.redLeftX; // adjusted
        public static int redLeftY = SquareConstant.redLeftY; // adjusted

        public static int redCenterX = SquareConstant.redCenterX; // adjusted
        public static int redCenterY = SquareConstant.redCenterY; // adjusted

        public static int width = SquareConstant.leftWidth; // adjusted
        public static int height = SquareConstant.leftHeight; // adjusted

        public static int centerHeight = SquareConstant.centerHeight;

        public static int centerWidth = SquareConstant.centerWidth;

        public static double redThreshold = 2.5; // unchanged
        public static double blueThreshold = 0.2; // unchanged
        public static double threshold = 0; // unchanged


        public double leftColor = 0.0;
        public double centerColor = 0.0;

        public Scalar left = new Scalar(0,0,0);
        public Scalar center = new Scalar(0,0,0);

        @Override
        public Mat processFrame(Mat input) {

            if (Location.side == Location.Side.RED) {
                threshold = redThreshold;
            } else {
                threshold = blueThreshold;
            }

            input.copyTo(finalMat);
            Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

            leftZoneArea = new Rect(Location.side == Location.Side.RED? redLeftX : blueLeftX, Location.side == Location.Side.RED? redLeftY : blueLeftY, width, height);
            centerZoneArea = new Rect(Location.side == Location.Side.RED?redCenterX:blueCenterX, Location.side == Location.Side.RED?redCenterY:blueCenterY, centerWidth, centerHeight);

            Mat leftZone = finalMat.submat(leftZoneArea);
            Mat centerZone = finalMat.submat(centerZoneArea);

            left = Core.sumElems(leftZone);
            center = Core.sumElems(centerZone);

            leftColor = left.val[0] / 1000000.0;
            centerColor = center.val[0] / 1000000.0;

            if(Location.side == Location.Side.BLUE){
                if (leftColor < threshold) {
                    // left zone has it
                    location = Location.Side.LEFT;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor < threshold) {
                    // center zone has it
                    location = Location.Side.CENTER;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                } else {
                    // right zone has it
                    location = Location.Side.RIGHT;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                }
            }else{
                if (leftColor > threshold) {
                    // left zone has it
                    location = Location.Side.CENTER;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                } else if (centerColor > threshold) {
                    // center zone has it
                    location = Location.Side.RIGHT;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                } else {
                    // right zone has it
                    location = Location.Side.LEFT;
                    Imgproc.rectangle(input, leftZoneArea, new Scalar(255, 255, 255));
                }
            }

            Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
            Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));

            lastFrame.set(input);

            leftZone.release();
            centerZone.release();

            return finalMat;
        }

        public Location.Side getLocation(){
            return location;
        }

        public double getLeftCenter(){
            return leftColor;
        }

        public double getCenterColor(){
            return centerColor;
        }
    }
}
