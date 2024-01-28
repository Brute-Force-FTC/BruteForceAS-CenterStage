package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Utilities.BruteForceRobot;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.exception.DuplicateNameException;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.exception.RobotProtocolException;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import java.io.PrintStream;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

//Brute Force Code

public class BruteForceRobot {
    public Orientation lastAngles = new Orientation();
    public BNO055IMU imu;
    private Blinker expansion_Hub_2;
    private Blinker controlHub;
    private HardwareDevice webcam_1;
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotor slideLeft;
    public DcMotor slideRight;
    public DcMotor intake;
    //public DcMotor testMotor;
    public CRServo intakeLeft;
    public CRServo intakeRight;
    public CRServo paperAirplane;
    //public CRServo testServo;

    static final double     COUNTS_PER_MOTOR_REV    = 1075.2 ;
    static final double     COUNTS_PER_REV          = 10;
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DRIVE_SPEED = 0.40;
    public static final double TURN_SPEED = 0.40;
    public static final double STRAFE_SPEED = 0.40;

    public BruteForceRobot(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        controlHub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        //testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        intakeLeft = hardwareMap.get(CRServo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "IntakeRight");
        paperAirplane = hardwareMap.get(CRServo.class, "paperAirplane");
        //testServo = hardwareMap.get(CRServo.class, "testServo");
        imu.initialize(parameters);
    }

    public double getAngle(double gA, Telemetry telemetry) {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        telemetry.addData("delta:", deltaAngle);

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        gA += deltaAngle;

        lastAngles = angles;

        return gA;
    }

    public void resetAngle(double gA) {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        gA = 0;
    }

    public double checkDirectionF(double gA, Telemetry telemetry) {
        //gain is how sensitive robot is to angle changes, aka how quickly it corrects
        double correction, angle, gain = 0.1;
        angle = getAngle(gA, telemetry);
        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }

    public double checkDirectionB(double gA, Telemetry telemetry) {
        //gain is how sensitive robot is to angle changes, aka how quickly it corrects
        double correction, angle, gain = 0.1;
        angle = getAngle(gA, telemetry);
        if (angle == 0) {
            correction = 0;
        } else {
            correction = angle;
        }

        correction = correction * gain;

        return correction;
    }

    public void resetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForward(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionF(gA, telemetry);
        frontLeft.setPower(-(speed-correction));
        frontRight.setPower((speed+correction));
        backLeft.setPower(-(speed-correction));
        backRight.setPower((speed+correction));
    }

    public void moveBackward(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionB(gA, telemetry);
        frontLeft.setPower((speed-correction));
        frontRight.setPower(-(speed+correction));
        backLeft.setPower((speed-correction));
        backRight.setPower(-(speed+correction));
    }

    public void moveRight(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionF(gA, telemetry);
        frontLeft.setPower(-(speed-correction));
        frontRight.setPower(-(speed+correction));
        backLeft.setPower((speed-correction));
        backRight.setPower((speed+correction));
    }

    public void moveLeft(double speed, double gA, Telemetry telemetry) {
        double correction = checkDirectionB(gA, telemetry);
        frontLeft.setPower((speed-correction));
        frontRight.setPower((speed+correction));
        backLeft.setPower(-(speed-correction));
        backRight.setPower(-(speed+correction));
    }

    public void moveVertical(double n) {
        frontLeft.setPower(n);
        frontRight.setPower(-n);
        backLeft.setPower(n);
        backRight.setPower(-n);
    }

    public void moveHorizontal(double n) {
        frontLeft.setPower(-n);
        frontRight.setPower(-n);
        backLeft.setPower(n);
        backRight.setPower(n);
    }

    public void rotate(double n) {
        frontLeft.setPower(-n);
        frontRight.setPower(-n);
        backLeft.setPower(-n);
        backRight.setPower(-n);
    }

    public void stopMoving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        slideLeft.setPower(0);
        slideRight.setPower(0);
        intake.setPower(0);
        //testMotor.setPower(0);
    }

    public void sleep(double n) {
        sleep(n);
    }

    public void encoderSL(int n) {
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setTargetPosition(n);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(0); //set to 1 or -1 depending on what power is needed to go up
        while (slideLeft.isBusy()) {
            ;
        }
        slideLeft.setPower(0);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderSR(int n) {
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setTargetPosition(n);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setPower(0); //set to 1 or -1 depending on what power is needed to go up
        while (slideRight.isBusy()) {
            ;
        }
        slideRight.setPower(0);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderSlide(int n) {
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setTargetPosition(n);
        slideRight.setTargetPosition(n);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setPower(-1);
        slideRight.setPower(1);
        while (slideLeft.isBusy()) {
            while (slideRight.isBusy()) {
                ;
            }
        }
        slideLeft.setPower(0);
        slideRight.setPower(0);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDriveIMU(String config, double gA, double Inches, double speed, boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;
            double correction = 0;

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", gA);
            telemetry.addData("3 correction", correction);

            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            }

            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (
                    frontLeft.isBusy()
                            && frontRight.isBusy()
                            && backLeft.isBusy()
                            && backRight.isBusy()
            )
            {
                if (config == "f") {
                    correction = checkDirectionF(gA, telemetry);
                    frontLeft.setPower(-(speed-correction));
                    frontRight.setPower((speed+correction));
                    backLeft.setPower(-(speed-correction));
                    backRight.setPower((speed+correction));
                } else if (config == "b") {
                    correction = checkDirectionB(gA, telemetry);
                    frontLeft.setPower((speed-correction));
                    frontRight.setPower(-(speed+correction));
                    backLeft.setPower((speed-correction));
                    backRight.setPower(-(speed+correction));
                } else if (config == "r") {
                    correction = checkDirectionF(gA, telemetry);
                    frontLeft.setPower(-(speed-correction));
                    frontRight.setPower(-(speed+correction));
                    backLeft.setPower((speed-correction));
                    backRight.setPower((speed+correction));
                } else if (config == "l") {
                    correction = checkDirectionB(gA, telemetry);
                    frontLeft.setPower((speed-correction));
                    frontRight.setPower((speed+correction));
                    backLeft.setPower(-(speed-correction));
                    backRight.setPower(-(speed+correction));
                }

                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderDrive(String config, double Inches, double speed, boolean opModeIsActive, Telemetry telemetry) {
        if (opModeIsActive) {
            double frontLeftInches = 0;
            double frontRightInches = 0;
            double backLeftInches = 0;
            double backRightInches = 0;

            if (config == "f") {
                frontLeftInches = -Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = Inches;
            } else if (config == "b") {
                frontLeftInches = Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = -Inches;
            } else if (config == "rl") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            } else if (config == "rr") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "l") {
                frontLeftInches = Inches;
                frontRightInches = Inches;
                backLeftInches = -Inches;
                backRightInches = -Inches;
            } else if (config == "r") {
                frontLeftInches = -Inches;
                frontRightInches = -Inches;
                backLeftInches = Inches;
                backRightInches = Inches;
            }

            int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            int newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (
                    frontLeft.isBusy()
                            && frontRight.isBusy()
                            && backLeft.isBusy()
                            && backRight.isBusy()
            )
            {
                telemetry.addData("COUNTS_PER_INCH = ", COUNTS_PER_INCH);
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());

                telemetry.update();
            }

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
