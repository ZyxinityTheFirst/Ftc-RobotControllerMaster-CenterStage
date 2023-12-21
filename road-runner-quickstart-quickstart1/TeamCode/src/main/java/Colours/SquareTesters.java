package Colours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "SquareTesters")
public class SquareTesters extends LinearOpMode {

    OpenCvWebcam webcam;
    SquareTesterPipeline pipeline1;

    @Override
    public void runOpMode() throws InterruptedException {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline1 = new SquareTesterPipeline();
        webcam.setPipeline(pipeline1);
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



        waitForStart();
        while (opModeIsActive()){
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

    public static class SquareTesterPipeline extends OpenCvPipeline {

        Rect leftZoneArea = new Rect(SquareConstant.redLeftX, SquareConstant.redLeftY, SquareConstant.width, SquareConstant.height);
        Rect centerZoneArea = new Rect(SquareConstant.redCenterX, SquareConstant.redCenterY, SquareConstant.width, SquareConstant.height);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input, leftZoneArea, new Scalar(255,255,255));
            Imgproc.rectangle(input, centerZoneArea, new Scalar(255,255,255));

            return input;
        }
    }
}
