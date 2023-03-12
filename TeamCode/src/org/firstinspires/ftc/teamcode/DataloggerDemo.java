package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.time.LocalDateTime;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Datalogger Demo", group = "Datalogger")
public class DataloggerDemo extends LinearOpMode {

    private Datalogger datalog;

    private Boolean isVirtualRobot = false;

    //wheel motors
    private DcMotor motorFrontLeft;     //3
    private DcMotor motorBackLeft;      //2
    private DcMotor motorFrontRight;    //0
    private DcMotor motorBackRight;     //1

    //timers
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime timeout = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // Instantiate datalog timer.
    private ElapsedTime dataTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    int logInterval = 10;               // target interval in milliseconds
    private  IMU imu;

    private DistanceSensor frontDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DistanceSensor backDistance;
    private ColorSensor colorSensor;

    double live_gyro_value = 0;
    double gyro_offset = 0;

    //finds optimal rotation
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2*Math.PI;

        }
        while (radians < -Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }


    //configure PID Controller for rotation
    double integralSum = 0;
    double lastError = 0;
    //Ku is 3
    double Kp = 1.2;
    double Ki = 0;
    double Kd = 0;

    //set endstate to a high number to prevent end from starting
    int endState = 10000;

    //PID control method
    public double PIDControl (double reference, double state) {
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;

    }

    private void logTurnData(double power, double directionInRadians, double error){
        telemetry.addData("power",String.format("%.6f", power));
        telemetry.addData("Kp",String.format("%.2f", Kp));
        telemetry.addData("Ki",String.format("%.2f", Ki));
        telemetry.addData("Kd",String.format("%.2f", Kd));
        telemetry.addData("directionInRadians",String.format("%.6f", directionInRadians));
        telemetry.addData("getHeadingInRadians",String.format("%.6f", getHeadingInRadians()));
        telemetry.addData("error",String.format("%.6f", error));

        datalog.addField((long) runtime.milliseconds());
        datalog.addField(String.format("%.6f", power));
        datalog.addField(String.format("%.2f", Kp));
        datalog.addField(String.format("%.2f", Ki));
        datalog.addField(String.format("%.2f", Kd));
        datalog.addField(String.format("%.6f", directionInRadians));
        datalog.addField(String.format("%.6f", getHeadingInRadians()));
        datalog.addField(String.format("%.6f", error));
        datalog.addField(timeout.milliseconds());
        datalog.newLine();
    }

    // Filter power
    private double filterPowerInTurn(double requestedPower){
        double filteredPower = requestedPower;
        double maxPower = 1;
        double minPower = 0.09;
        if (isVirtualRobot){
            minPower = 0.05;
        }
        if ((requestedPower < minPower) && (requestedPower > 0)) {
            filteredPower = minPower;
        } else if ((requestedPower > -minPower) && (requestedPower < 0)) {
            filteredPower = -minPower;
        } else if (filteredPower > maxPower) {
            filteredPower=maxPower;
        } else if (filteredPower <-maxPower) {
            filteredPower=-maxPower;
        }
        return filteredPower;
    }

    // For all axes, IMU angles are provided in the range of -180 to +180 degrees (or from -Pi to +Pi radians).
    // If you are working with values that might cross the +/- 180-degree transition, handle this with your programming
    // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
    // Orthogonal orientation of Hub - Logo RIGHT, USB BACKWARD

    IMU.Parameters myIMUparameters;


    // Turns Robot in direction specified using PID Controller and IMU.
    private void turnByIMU(double directionInDegrees) {
        double directionInRadians = Math.toRadians(directionInDegrees);
        double lastHeading = getHeadingInRadians();
        double error = angleWrap(directionInRadians - lastHeading);
        integralSum = 0;
        lastError = 0;
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Reset timer for datalogging interval.
        dataTimer.reset();
        timeout.reset();
        loopTimer.reset();
        double tick = loopTimer.milliseconds();
        double power = PIDControl(directionInRadians, lastHeading);
        telemetry.addLine("turning");
        telemetry.update();
        while (((Math.abs(error) > Math.toRadians(1))) && opModeIsActive()) {
            if (loopTimer.milliseconds() >= (tick + 10)){
                tick = loopTimer.milliseconds();
                lastHeading = getHeadingInRadians();
                power = PIDControl(directionInRadians, getHeadingInRadians());
                power = filterPowerInTurn(power);
                motorFrontLeft.setPower(power);
                motorBackLeft.setPower(power);
                motorFrontRight.setPower(-power);
                motorBackRight.setPower(-power);
                error = angleWrap(directionInRadians - lastHeading);
                if (dataTimer.time() >= logInterval) {
                    logTurnData(power,directionInRadians,error);
                    dataTimer.reset();      // start the interval timer again
                    updateTelemetry();
                }
                if (timeout.seconds()> 5) {
                    break;
                }

            }
        }
        logTurnData(power,directionInRadians,error);
        telemetry.addLine("rotation complete after " + timeout.time()  + " seconds");
        updateTelemetry();

        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);

        RobotLog.ii("DbgLog", "Turn Complete after " + timeout.time()  + "ms: RAD=" +  getHeadingInRadians() + " DEG="+ Math.toDegrees(getHeadingInRadians())+" POWER="+power);

    }


    //points in direction specified using PID Controller and IMU.
    private void pointDirectionV2(double direction) {
        double Ku = 1;
        double Tu = 1;
        Boolean isPIDZieglerNicholsMethod = false;
        if (isPIDZieglerNicholsMethod){
            Kp = 0.6 * Ku;
            Ki = 1.2 * Ku/Tu;
            Kd = 0.075 * Ku * Tu;
        } else {
            Kp = 1.2;
            Ki = 0.1;
            Kd = 0.1;
        }
        turnByIMU(direction);
    }

    //points in direction specified using PID Controller and IMU.
    private void pointDirection(double direction) {
        double Ku = 3;
        Kp = 1.2;
        Ki = 0;
        Kd = 0;
        turnByIMU(direction);
    }

    private double getHeadingInRadians(){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -orientation.firstAngle;
    }
    private double getHeadingInDegrees(){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -orientation.firstAngle * 180.0 / Math.PI;
    }
    private void updateTelemetry(){
        telemetry.addData("Color","R %d  G %d  B %d", colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Heading", " %.1f", getHeadingInDegrees());
        telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        telemetry.addData("Front Distance", " %.1f", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", " %.1f", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", " %.1f", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Distance", " %.1f", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoders"," %d %d %d %d", motorBackLeft.getCurrentPosition(), motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(), motorBackRight.getCurrentPosition());
        telemetry.update();
    }

    public void runOpMode(){
        isVirtualRobot = System.getProperty("os.name").contains("Windows");

        // Speed Variable
        double speed = 0.5;

        LocalDateTime myDateObj = LocalDateTime.now();
        String filename = "Datalogger Demo" + myDateObj.toString().replace(":","." ) + ".csv";
        datalog = new Datalogger(filename );
        // Name the fields (column labels) generated by this OpMode.
        datalog.addField("Runtime (ms)");
        datalog.addField("Power");
        datalog.addField("PID Kp");
        datalog.addField("PID Ki");
        datalog.addField("PID Kd");
        datalog.addField("PID Target Heading");
        datalog.addField("PID Reference Heading");
        datalog.addField("PID Error");
        datalog.addField("PID Timer");
        datalog.firstLine();                        // end first line (row)
        telemetry.addData("Datalogger started." , "");
        telemetry.addData("file", filename);
        motorBackLeft = hardwareMap.dcMotor.get("back_left_motor");
        motorFrontLeft = hardwareMap.dcMotor.get("front_left_motor");
        motorFrontRight = hardwareMap.dcMotor.get("front_right_motor");
        motorBackRight= hardwareMap.dcMotor.get("back_right_motor");
        if (isVirtualRobot) {
            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        }else {
            motorBackRight.setDirection(DcMotor.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        }
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()){

            if (gamepad1.dpad_up){
                pointDirection(0);
            } else if (gamepad1.dpad_down) {
                pointDirection(180);
            } else if (gamepad1.dpad_left) {
                pointDirection(-90);
            } else if (gamepad1.dpad_right) {
                pointDirection(90);
            } else if (gamepad1.y){
                pointDirectionV2(0);
            } else if (gamepad1.a) {
                pointDirectionV2(180);
            } else if (gamepad1.x) {
                pointDirectionV2(-90);
            } else if (gamepad1.b) {
                pointDirectionV2(90);
            } else {

                double y = -gamepad1.left_stick_y;          // Up-Down
                double x = gamepad1.left_stick_x * 1.1;    // Left-Right
                double rx = gamepad1.right_stick_x;         //Rotation

                // Read inverse IMU heading, as the IMU heading is CW positive
                Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
                live_gyro_value = -orientation.firstAngle * 180.0 / Math.PI;
                double botHeading = live_gyro_value - gyro_offset;

                //double botHeading = -imu.getAngularOrientation().firstAngle;

                double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
                double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;

                frontLeftPower = frontLeftPower * speed;
                frontRightPower = frontRightPower * speed;
                backLeftPower = backLeftPower * speed;
                backRightPower = backRightPower * speed;

                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
                if (y != 0 || x != 0 || rx != 0){
                    updateTelemetry();
                }

            }

        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        datalog.closeDataLogger();
    }
}