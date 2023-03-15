package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.time.LocalDateTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@Config
@TeleOp(name = "Datalogger Demo With Dashboard", group = "Datalogger")
public class DataloggerDemoWithDashboard extends LinearOpMode {

    private Datalogger datalog;

    private Boolean isVirtualRobot = false;

    //wheel motors
    private DcMotor motorFrontLeft;     //3
    private DcMotor motorBackLeft;      //2
    private DcMotor motorFrontRight;    //0
    private DcMotor motorBackRight;     //1

    private DcMotor MotorSlide;
    private CRServo right;
    private CRServo left;

    public static double squeezeFactor = 0.8;

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
    private VoltageSensor battery;

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
    public static double Kp = 1;
    public static double Ki = 0.05;
    public static double Kd = 0.01;
    public static int degreeErrorAllowed = 4;
    public static int tickUpdateInMs = 100;

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

    public void PIDReset(){
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    private void logDataPoint(double timeout, double runtime, double power,
                              double directionInRadians, double headingInRadians,
                              double errorInRadians, double Kp, double Ki, double Kd,double battery) {
        dataPoints[dataPointCount][0]=timeout;
        dataPoints[dataPointCount][1]=runtime;
        dataPoints[dataPointCount][2]=power;
        dataPoints[dataPointCount][3]=directionInRadians;
        dataPoints[dataPointCount][4]=headingInRadians;
        dataPoints[dataPointCount][5]=errorInRadians;
        dataPoints[dataPointCount][6]=Kp;
        dataPoints[dataPointCount][7]=Ki;
        dataPoints[dataPointCount][8]=Kd;
        dataPoints[dataPointCount][9]=battery;
        dataPointCount++;
        updateTelemetry();
    }
    private void logTurnData(){
        for (int i = 0; i < dataPointCount; i++) {
            datalog.addField((long) dataPoints[i][0]);
            datalog.addField((long) dataPoints[i][1]);
            datalog.addField(String.format("%.6f", dataPoints[i][2]));
            datalog.addField(String.format("%.6f", dataPoints[i][3]));
            datalog.addField(String.format("%.6f", dataPoints[i][4]));
            datalog.addField(String.format("%.6f", dataPoints[i][5]));
            datalog.addField(String.format("%.2f", dataPoints[i][6]));
            datalog.addField(String.format("%.2f", dataPoints[i][7]));
            datalog.addField(String.format("%.2f", dataPoints[i][8]));
            datalog.addField(String.format("%.2f", dataPoints[i][9]));
            datalog.newLine();
        }
    }

    public static double maxPower = 1.0;
    public static double minPower = 0.10;

    // Filter power
    private double filterPowerInTurn(double requestedPower){
        double filteredPower = requestedPower;
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

    int rows = 30000, columns = 10;
    int dataPointCount = 0;

    private double[][] dataPoints;
    private void initializeDataPoints(){
        dataPoints = new double[rows][columns];
        // initializing the array elements using for loop
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < columns; j++) {
                dataPoints[i][j] = 0;
            }
        }
        dataPointCount = 0;
    }

    private double setMotorPowerInTurn(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(-power);
        return power;
    }

    // Turns Robot in direction specified using PID Controller and IMU.
    private void turnByIMU(double directionInDegrees) {
        double directionInRadians = Math.toRadians(directionInDegrees);
        double lastHeading = getHeadingInRadians();
        double error = angleWrap(directionInRadians - lastHeading);
        int foundTargetCount=0;
        double batteryVoltage=12;
        Boolean isTargetFound=false;
        initializeDataPoints();
        PIDReset();
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
        while (!isTargetFound && opModeIsActive()) {

            if (loopTimer.milliseconds() >= (tick + tickUpdateInMs)){
                tick += tickUpdateInMs;
                lastHeading = getHeadingInRadians();
                power = PIDControl(directionInRadians, getHeadingInRadians());
                power = setMotorPowerInTurn(filterPowerInTurn(power));
                error = angleWrap(directionInRadians - lastHeading);
                if (!isVirtualRobot) {
                    batteryVoltage = battery.getVoltage();
                }
                logDataPoint(timeout.milliseconds(),runtime.milliseconds(),power,directionInRadians,getHeadingInRadians(),error,Kp,Ki,Kd,batteryVoltage);
                if (Math.abs(error) < Math.toRadians(Math.abs(degreeErrorAllowed))){
                    foundTargetCount++;
                    if (foundTargetCount > 2){
                        isTargetFound = true;
                    }
                }
                if (timeout.seconds()> 5) {
                    break;
                }

            }
        }
        logTurnData();
        telemetry.addLine("rotation complete after " + timeout.time()  + " seconds");
        updateTelemetry();
        setMotorPowerInTurn(0);

        RobotLog.ii("DbgLog", "Turn Complete after " + timeout.time()  + "ms: RAD=" +  getHeadingInRadians() + " DEG="+ Math.toDegrees(getHeadingInRadians())+" POWER="+power);

    }


    //points in direction specified using PID Controller and IMU.
    private void pointDirectionV2(double direction) {
        double Ku = 1.5;
        double Tu = 2;
        Boolean isPIDZieglerNicholsMethod = false;
        if (isPIDZieglerNicholsMethod){
            Kp = 0.6 * Ku;
            Ki = 1.2 * Ku/Tu;
            Kd = 0.075 * Ku * Tu;
        } else {
            Kp = 1.5;
            Ki = 0.1;
            Kd = 0.1;
        }
        turnByIMU(direction);
    }

    //points in direction specified using PID Controller and IMU.
    private void pointDirection(double direction) {
        // Use values set by default or FTC Dashboard
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
        telemetry.addData("Heading", getHeadingInDegrees());
        telemetry.addData("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
        telemetry.addData("Front Distance", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Left Distance", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Right Distance", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Distance", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("motorBackLeftEncoder",motorBackLeft.getCurrentPosition());
        telemetry.addData("motorFrontLeftEncoder",motorFrontLeft.getCurrentPosition());
        telemetry.addData("motorFrontRightEncoder",motorFrontRight.getCurrentPosition());
        telemetry.addData("motorBackRightEncoder",motorBackRight.getCurrentPosition());
        if (!isVirtualRobot) {
            telemetry.addData("Battery Voltage", battery.getVoltage());
        }
        telemetry.update();
    }

    private void initializeTelemetry(){
        String filename;
        if (isVirtualRobot){
            LocalDateTime myDateObj = LocalDateTime.now();
            filename = "Datalogger Demo" + myDateObj.toString().replace(":","." ) + ".csv";
        } else {
            filename = "Datalogger Demo" + ".csv";
        }
        datalog = new Datalogger(filename );
        // Name the fields (column labels) generated by this OpMode.
        datalog.addField("PID Timer");
        datalog.addField("Runtime (ms)");
        datalog.addField("Power");
        datalog.addField("PID Target Heading");
        datalog.addField("PID Reference Heading");
        datalog.addField("PID Error");
        datalog.addField("PID Kp");
        datalog.addField("PID Ki");
        datalog.addField("PID Kd");
        datalog.addField("Battery Voltage");
        datalog.firstLine();                        // end first line (row)
        if (!isVirtualRobot) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = dashboard.getTelemetry();
        }
        telemetry.addData("Datalogger started." , "");
        telemetry.addData("file", filename);
        telemetry.addData("gamepad1 controls", "function");
        telemetry.addData("left joystick" , "field centric movement");
        telemetry.addData("right joystick" , "field centric rotate");
        telemetry.addData("dpad" , "field centric quick turn");
        telemetry.addData("-x" , "imu gyro reset");
        telemetry.addData("-a" , "accel boost");
        telemetry.addData("-b" , "TBD");
        telemetry.addData("-y" , "TBD");
        telemetry.addData("-LT" , "slider down");
        telemetry.addData("-LB" , "slider up");
        telemetry.addData("-RT" , "claw close and hold");
        telemetry.addData("-RB" , "claw open");
        telemetry.addData("gamepad2 controls", "function");
        telemetry.addData("-left joystick" , "field centric movement");
        telemetry.addData("-right joystick" , "slider movement");
        telemetry.addData("-x" , "slider to top");
        telemetry.addData("-a" , "slider setpoint");
        telemetry.addData("-b" , "slider to bottom");
        telemetry.addData("-y" , "slider other");
        telemetry.addData("-RT" , "claw close then release");
        telemetry.addData("-RB" , "TBD");
    }

    public void runOpMode(){
        isVirtualRobot = System.getProperty("os.name").contains("Windows");

        // Speed Variable
        double speed = 0.5;
        initializeTelemetry();

        if (isVirtualRobot) {
            motorBackLeft = hardwareMap.dcMotor.get("back_left_motor");
            motorFrontLeft = hardwareMap.dcMotor.get("front_left_motor");
            motorFrontRight = hardwareMap.dcMotor.get("front_right_motor");
            motorBackRight= hardwareMap.dcMotor.get("back_right_motor");
            MotorSlide = hardwareMap.get(DcMotor.class, "arm_motor");
            MotorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //right = hardwareMap.get(CRServo.class, "right_servo");
            //left = hardwareMap.get(CRServo.class, "left_servo");
        }else {
            motorBackLeft = hardwareMap.dcMotor.get("Motor2");
            motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
            motorFrontRight = hardwareMap.dcMotor.get("Motor0");
            motorBackRight= hardwareMap.dcMotor.get("Motor1");
            MotorSlide = hardwareMap.get(DcMotor.class, "MotorSlide");
            MotorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right = hardwareMap.get(CRServo.class, "right");
            left = hardwareMap.get(CRServo.class, "left");
        }
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (isVirtualRobot){
            frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
            leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
            rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
            backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
            colorSensor = hardwareMap.colorSensor.get("color_sensor");
        } else {
            frontDistance = hardwareMap.get(DistanceSensor.class, "arm distance");
            leftDistance = hardwareMap.get(DistanceSensor.class, "control distance");
            rightDistance = hardwareMap.get(DistanceSensor.class, "arm distance");
            backDistance = hardwareMap.get(DistanceSensor.class, "control distance");
            colorSensor = hardwareMap.colorSensor.get("color1");
            battery = hardwareMap.voltageSensor.get("Control Hub");
        }

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        //RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        //RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        telemetry.setMsTransmissionInterval(50);

        //ARM INIT
        int encoderPosition;
        int lastEncoderPosition;
        int encoderReal;
        int encoderMin;
        int encoderMax;
        int pole1;
        int pole2;
        int pole3;
        int stack;
        int dist1=0;
        int dist2=0;
        int dist3=0;
        int dist4=0;

        int toggleLeftTrigger=0;



        encoderPosition = 0;
        encoderMin = -4411;
        encoderMax = 0;
        pole1 = -1845;
        pole2 = -2900;
        pole3 = encoderMin+20;
        stack = -670;


        double live_gyro_value = 0;
        double gyro_offset = 0;
        double lastMaxSqueeze = 0;
        double lastMaxLift = 0;

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
            } else {
                encoderReal = (int) Math.round(MotorSlide.getCurrentPosition());
                if (gamepad1.right_trigger > lastMaxSqueeze){
                    lastMaxSqueeze = gamepad1.right_trigger;
                    if (isVirtualRobot) {
                        //left.setPower(gamepad1.right_trigger * squeezeFactor - 0.7);
                        //right.setPower(gamepad1.right_trigger * -squeezeFactor + 0.7);
                    } else {
                        if (isVirtualRobot) {
                            //left.setPower(gamepad1.right_trigger * squeezeFactor - 0.7);
                            //right.setPower(gamepad1.right_trigger * -squeezeFactor + 0.7);
                        } else {
                            left.setPower(gamepad1.right_trigger * squeezeFactor - 0.7);
                            right.setPower(gamepad1.right_trigger * -squeezeFactor + 0.7);
                        }

                    }

                } else if (gamepad1.right_bumper){
                    lastMaxSqueeze = 0;
                    if (isVirtualRobot){
                        //left.setPower(-0.7);
                        //right.setPower(0.7);
                    } else {
                        left.setPower(-0.7);
                        right.setPower(0.7);
                    }

                } else if (lastMaxSqueeze <= 0){
                    if (isVirtualRobot) {
                        if (encoderReal<stack) {
                            //left.setPower(gamepad2.right_trigger * squeezeFactor - 0.7);
                            //right.setPower(gamepad2.right_trigger * -squeezeFactor + 0.7);
                        } else {
                            //left.setPower(gamepad2.right_trigger * 0.5 - 0.25);
                            //right.setPower(gamepad2.right_trigger * -0.5 + 0.25);
                        }
                    }else {
                        if (encoderReal<stack) {
                            left.setPower(gamepad2.right_trigger * squeezeFactor - 0.7);
                            right.setPower(gamepad2.right_trigger * -squeezeFactor + 0.7);
                        } else {
                            left.setPower(gamepad2.right_trigger * 0.5 - 0.25);
                            right.setPower(gamepad2.right_trigger * -0.5 + 0.25);
                        }
                    }

                }
                if (gamepad1.left_trigger > lastMaxLift){
                    lastMaxLift = gamepad1.left_trigger;
                    encoderPosition = encoderMax - ((int)((encoderMax - encoderMin) * gamepad1.left_trigger));
                } else if (gamepad1.left_bumper){
                    encoderPosition = encoderMax;
                    lastMaxLift = 0;
                } else if (lastMaxLift <= 0){
                    if (Math.abs(gamepad2.right_stick_y) > 0.1){
                        encoderPosition = encoderReal + (int)Math.round(gamepad2.right_stick_y) * 150;
                    }
                }



                if(toggleLeftTrigger==1 && gamepad2.left_trigger<0.5){
                    toggleLeftTrigger=0;
                    encoderMin = -4411+encoderReal-10;
                    encoderMax = 0+encoderReal-10;
                    pole1 = -1845+encoderReal-10;
                    pole2 = -2900+encoderReal-10;
                    pole3 = encoderMin+20-10;
                    stack = -670+encoderReal-10;
                }
                if(gamepad2.left_trigger>0.5){
                    toggleLeftTrigger=1;
                } else {
                    toggleLeftTrigger=0;
                    if (encoderPosition < encoderMin) {
                        encoderPosition = encoderMin;
                    }
                    if (encoderPosition > encoderMax) {
                        encoderPosition = encoderMax;
                    }
                }
                dist1 = Math.abs(encoderReal - pole1);
                dist2 = Math.abs(encoderReal - pole2);
                dist3 = Math.abs(encoderReal - pole3);
                dist4 = Math.abs(encoderReal - stack);

                if(gamepad2.a) {
                    if ((dist4 < dist1) && (dist4 < dist2) && (dist4 < dist3)) {
                        encoderPosition = stack;
                    }
                    if ((dist3 < dist1) && (dist3 < dist2) && (dist3 < dist4)) {
                        encoderPosition = pole3;
                    }
                    if ((dist2 < dist1) && (dist2 < dist4) && (dist2 < dist3)) {
                        encoderPosition = pole2;
                    }
                    if ((dist1 < dist4) && (dist1 < dist2) && (dist1 < dist3)) {
                        encoderPosition = pole1;
                    }
                }

                if(gamepad2.x) {
                    encoderPosition = pole3;
                }
                if(gamepad2.b) {
                    encoderPosition = encoderMax;
                }

                MotorSlide.setTargetPosition(encoderPosition);
                MotorSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) MotorSlide).setTargetPositionTolerance(10);
                ((DcMotorEx) MotorSlide).setVelocity(2000);
                if (encoderReal < pole2+200){
                    speed = 0.4;
                } else if(gamepad1.a) {
                    speed = 1;
                } else {
                    speed = 0.5;
                }

                if(gamepad1.x) {
                    gyro_offset = live_gyro_value;
                }
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
                updateTelemetry();
            }

        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        datalog.closeDataLogger();
    }
}
