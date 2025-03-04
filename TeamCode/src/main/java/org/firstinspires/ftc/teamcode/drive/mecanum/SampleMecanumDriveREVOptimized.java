package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;

/*
 * Optimized mecanum drive implementation for REV ExHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    public ExpansionHubEx hub, hub2;
    private ExpansionHubMotor leftFront, leftRear, rightRear, rightFront;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor LeftIntake;
    public ExpansionHubMotor RightIntake;
    public ExpansionHubServo Gripper;
    public ExpansionHubServo Wrist;
    public ExpansionHubServo Elbow;
    public ExpansionHubServo RightHook;
    public ExpansionHubServo LeftHook;
    public ExpansionHubServo LeftAngle = null;
    public ExpansionHubServo RightAngle = null;

    public double GripperOpen = 1;
    public double GripperClosed = .64;

    public double ElbowCollectionPosition = .04;                                  // Dpad_up
    public double ElbowBackLeftDepositPosition = .59;                             // B and X
    public double ElbowFrontRightDepositPosition = .47;                           // A and Y

    public double ElbowExtended = 1600;
    public double ElbowRetract = 0.25;
    public double ElbowExtend = 0.75;

    public double WristCollectionPosition = .11;                                  // Dpad_Up
    public double WristBackDepositPosition = .11;                                 // B
    public double WristFrontDepositPosition = .767;                                  // Y
    //    public double WristLeftDepositPosition = 1;                                 // X
    public double WristRightDepositPosition = .43;                                  // A

    public double LeftHookDisengaged = .42;                                          //tbd
    public double LeftHookEngaged = .12;                                             // tbd

    public double RightHookDisengaged = .32;                                         //tbd
    public double RightHookEngaged = .62;                                            // tbd

    public double LeftAngleOpen = 0.69833;
    public double LeftAngleIntake = 0.71722;
    public double LeftAngleGripped = 0.72722;
    public double LeftAngleScanning = .71722;

    public double RightAngleOpen = 0.80777;
    public double RightAngleIntake = 0.79333;
    public double RightAngleGripped = 0.76944;

    double SpoolDiameterIN = 1.25;
    double LiftMotorTicksPerRotationofOuputShaft = 537.6;         // for gobilda 19.2:1 Motor

    public double LiftTicksPerInch = LiftMotorTicksPerRotationofOuputShaft / (SpoolDiameterIN * Math.PI);

    public double MinimumElbowMovementHeightIN = 6;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        // TODO: adjust the names of the following hardware devices to match your configuration
        // for simplicity, we assume that the desired IMU and drive motors are on the same hub
        // if your motors are split between hubs, **you will need to add another bulk read**
        hub = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub1");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "ExpansionHub2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.get(ExpansionHubMotor.class, "FrontLeft");
        leftRear = hardwareMap.get(ExpansionHubMotor.class, "BackLeft");
        rightRear = hardwareMap.get(ExpansionHubMotor.class, "BackRight");
        rightFront = hardwareMap.get(ExpansionHubMotor.class, "FrontRight");


        LeftLift = hardwareMap.get(ExpansionHubMotor.class, "LeftLift");
        RightLift = hardwareMap.get(ExpansionHubMotor.class, "RightLift");
        LeftIntake = hardwareMap.get(ExpansionHubMotor.class, "LeftIntake");
        RightIntake = hardwareMap.get(ExpansionHubMotor.class, "RightIntake");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        LeftLift.setTargetPosition(LeftLift.getCurrentPosition());
        LeftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLift.setTargetPosition(RightLift.getCurrentPosition());
        RightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftLift.setDirection(DcMotor.Direction.REVERSE);
        LeftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftIntake.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        Gripper = hardwareMap.get(ExpansionHubServo.class, "Gripper");
        Wrist = hardwareMap.get(ExpansionHubServo.class, "Wrist");
        Elbow = hardwareMap.get(ExpansionHubServo.class, "Elbow");
        LeftHook = hardwareMap.get(ExpansionHubServo.class, "LeftHook");
        RightHook = hardwareMap.get(ExpansionHubServo.class, "RightHook");
        LeftAngle = hardwareMap.get(ExpansionHubServo.class, "LeftAngle");
        RightAngle = hardwareMap.get(ExpansionHubServo.class, "RightAngle");

        LeftHook.setPosition(LeftHookDisengaged);
        RightHook.setPosition(RightHookDisengaged);

        // TODO: if desired, use setLocalizer() to change the localization method
//        setLocalizer(new MecanumLocalizer(this, true));
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelPositions = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelPositions.add(encoderTicksToInches(bulkData.getMotorCurrentPosition(motor)));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();

        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0, 0.0);
        }

        List<Double> wheelVelocities = new ArrayList<>();
        for (ExpansionHubMotor motor : motors) {
            wheelVelocities.add(encoderTicksToInches(bulkData.getMotorVelocity(motor)));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
