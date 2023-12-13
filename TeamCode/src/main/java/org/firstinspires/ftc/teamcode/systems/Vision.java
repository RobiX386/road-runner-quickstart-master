package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Vision {
    public enum CameraCase {
        LEFT,
        CENTER,
        RIGHT
    }

    // 2 cazuri
    public static Scalar BCONE = new Scalar(0, 0, 255);
    public static Scalar RCONE = new Scalar(255, 0, 0);
    public static Scalar CURRENTCONE = BCONE;

    public static int cameraStop = 0;

    static public class TeamPropPipeline extends OpenCvPipeline {
        public static CameraCase cc = CameraCase.CENTER;
        public final Scalar BLUE = new Scalar(0, 0, 255);
        public final Scalar BLACK = new Scalar(0, 0, 0);

        final Point REGION1 = new Point(0,250);
        final Point REGION2 = new Point(700,250);
        final Point REGION3 = new Point(1420,250);

        static final int REGION_W = 500;
        static final int REGION_H = 450;

        static final int FONT = Imgproc.FONT_HERSHEY_COMPLEX;

        Point reg1_A = new Point(REGION1.x, REGION1.y);
        Point reg1_B = new Point(REGION1.x+REGION_W, REGION1.y+REGION_H);
        Point reg2_A = new Point(REGION2.x, REGION2.y);
        Point reg2_B = new Point(REGION2.x+REGION_W, REGION2.y+REGION_H);
        Point reg3_A = new Point(REGION3.x, REGION3.y);
        Point reg3_B = new Point(REGION3.x+REGION_W, REGION3.y+REGION_H);

        //...
        // cod detectie
        @Override
        public void init(Mat firstFrame) {
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat reg1 = input.submat(new Rect(reg1_A, reg1_B));
            Mat reg2 = input.submat(new Rect(reg2_A, reg2_B));
            Mat reg3 = input.submat(new Rect(reg3_A, reg3_B));
            Scalar avg1 = Core.mean(reg1);
            Scalar avg2 = Core.mean(reg2);
            Scalar avg3 = Core.mean(reg3);
            double dist1 = Math.sqrt(
                    Math.pow(avg1.val[0]-CURRENTCONE.val[0], 2) +
                            Math.pow(avg1.val[1]-CURRENTCONE.val[1], 2) +
                            Math.pow(avg1.val[2]-CURRENTCONE.val[2], 2) );
            double dist2 = Math.sqrt(
                    Math.pow(avg2.val[0]-CURRENTCONE.val[0], 2) +
                            Math.pow(avg2.val[1]-CURRENTCONE.val[1], 2) +
                            Math.pow(avg2.val[2]-CURRENTCONE.val[2], 2) );
            double dist3 = Math.sqrt(
                    Math.pow(avg3.val[0]-CURRENTCONE.val[0], 2) +
                            Math.pow(avg3.val[1]-CURRENTCONE.val[1], 2) +
                            Math.pow(avg3.val[2]-CURRENTCONE.val[2], 2) );
            if (dist1 < dist2 && dist1 < dist3) {
                Imgproc.rectangle(input, reg1_A, reg1_B, BLUE, 2);
                Imgproc.putText(input, "LEFT", reg1_A, FONT, 1, BLACK, 1);
                cc = CameraCase.LEFT;
            } else if (dist2 < dist1 && dist2 < dist3) {
                Imgproc.rectangle(input, reg2_A, reg2_B, BLUE, 2);
                Imgproc.putText(input, "CENTER", reg2_A, FONT, 1, BLACK, 1);
                cc = CameraCase.CENTER;
            } else if (dist3 < dist1 && dist3 < dist2) {
                Imgproc.rectangle(input, reg3_A, reg3_B, BLUE, 2);
                Imgproc.putText(input, "RIGHT", reg3_A, FONT, 1, BLACK, 1);
                cc = CameraCase.RIGHT;
            }
            return input;
        }
    }

    public static void scanTeamProp(HardwareMap hardwareMap, Scalar col) {
        OpenCvWebcam webcam;
        TeamPropPipeline pipeline;

        /// Se schimba de aici numele camerei
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorView", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;

        CURRENTCONE = col;

        webcamName = hardwareMap.get(WebcamName.class, "WebcamMain");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        if(cameraStop == 1) {
            Thread closeCameraThread = new Thread(() -> webcam.closeCameraDevice());
            closeCameraThread.start();
        }
        else {
            pipeline = new TeamPropPipeline();
            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                    //telemetry.addLine("Opened camera");
                    //telemetry.update();
                }

                @Override
                public void onError(int errorCode) {
                    //telemetry.addLine("There has been camera error :(");
                    //telemetry.update();
                }
            });
        }
    }
}