package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.util.List;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;


public class TensorFlowUtils {

    private Blinker controlHub;
    private Blinker expansionHub;
    private Gyroscope imu;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_ASSET = "model_3.tflite";
    private static final String[] LABELS = {
            "BlueTSE",
            "RedTSE",
            "expansion_Hub_2; private Blinker controlHub; private HardwareDevice webcam_1;",
    };
    private TfodProcessor tfod;

    public VisionPortal visionPortal;

    public TensorFlowUtils() {
    }

    public TensorFlowUtils(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansionHub = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        initTfod(hardwareMap);
    }

    //public VuforiaLocalizer getVuforia() {
    //  return vuforia;
    //}


    /*public int returnTSEPosition(boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            //while (opModeIsActive) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        if (
                                recognition.getLabel() == "Pink Squares"
                        )
                        {
                            return 1;
                        } else if (
                                recognition.getLabel() == "Purple Triangles"
                        )
                        {
                            return 2;
                        } else if (
                                recognition.getLabel() == "Green Circles"
                        )
                        {
                            return 3;
                        }
                    }
                    telemetry.addLine("Outside for loop");
                    telemetry.update();
                }
            }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return 3;
    }*/

    private void initTfod(HardwareMap hardwareMap) {
        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);
    }

    public void telemetryTfod(Telemetry telemetry) {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

    public int returnRDA(boolean opModeIsActive, Telemetry telemetry) {
        int q = 0;
        if (opModeIsActive) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Object Detected", currentRecognitions.size());
                int i = 0;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());

                    if (recognition.getLabel() == "RedTSE") {
                        if (x <= 200 && x >= 0) {
                            q = 1;
                        } else {
                            q = 2;
                        }
                    } else {
                        q = 3;
                    }

                    i++;
                }
            } else {
                q = 3;
            }
        }
        return q;
    }

    public int returnTSEPositionRDA(boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            //while (opModeIsActive) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Object Detected", currentRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel() == "expansion_Hub_2; private Blinker controlHub; private HardwareDevice webcam_1;") {
                        if (x <= 200 && x >= 0) {
                            return 1;
                        } else {
                            return 2;
                        }
                    } else {
                        return 3;
                    }
                }
                telemetry.addLine("Outside for loop");
                telemetry.update();
            }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return 3;
    }

    public int returnTSEPositionRPA(boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            //while (opModeIsActive) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Object Detected", currentRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel() == "expansion_Hub_2; private Blinker controlHub; private HardwareDevice webcam_1;") {
                        if (x <= 200 && x >= 0) {
                            return 1;
                        } else {
                            return 2;
                        }
                    } else {
                        return 3;
                    }
                }
                telemetry.addLine("Outside for loop");
                telemetry.update();
            }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return 3;
    }

    public int returnTSEPositionBDA(boolean opModeIsActive, Telemetry telemetry) {
        int q = 0;
        if (opModeIsActive) {
            //while (opModeIsActive) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Object Detected", currentRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel() == "RedTSE") {
                        if (x >= 0 && x <= 250) {
                            q = 1;
                        } else {
                            q = 2;
                        }
                    } else {
                        q = 3;
                    }
                }
                telemetry.addLine("Outside for loop");
                telemetry.update();
            }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return q;
    }

    public int returnTSEPositionBPA(boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            //while (opModeIsActive) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            if (currentRecognitions != null) {
                telemetry.addData("# Object Detected", currentRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : currentRecognitions) {
                    double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                    double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;

                    if (recognition.getLabel() == "RedTSE") {
                        if (x <= 200 && x >= 0) {
                            return 1;
                        } else {
                            return 2;
                        }
                    } else {
                        return 3;
                    }
                }
                telemetry.addLine("Outside for loop");
                telemetry.update();
            }
            //}
        }
        telemetry.addLine("You are here");
        telemetry.update();
        return 3;
    }



}






















