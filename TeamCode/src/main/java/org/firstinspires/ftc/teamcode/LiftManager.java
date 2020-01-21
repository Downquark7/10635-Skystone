package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.ExpansionHubServo;
import org.openftc.revextensions2.RevBulkData;

public class LiftManager {
    public ExpansionHubMotor LeftLift;
    public ExpansionHubMotor RightLift;
    public ExpansionHubMotor SlideEncoder;
    public ExpansionHubServo Elbow;
    public double liftTargetIN = 0;
    public double slideTargetIN = 0;
    public boolean isBusy = false;
    public double liftPower = 1;
    public double pidPower = 0.1;
    public double LiftTicksPerInch = 537.6 * 1.25 * Math.PI;     // for gobilda 19.7:1 and 1.25 inch spool
    public double SlideTicksPerInch = 360 * 1.25 * Math.PI;      // for vex optical shaft encoder and 1.25 inch spool
    public double tolerance = 0.5;
    public double LiftPositionIN = 0;
    public double SlidePositionIN = 0;

    public LiftManager(ExpansionHubMotor LeftLift, ExpansionHubMotor RightLift, ExpansionHubServo Elbow, ExpansionHubMotor slideEncoder) {
        this.LeftLift = LeftLift;
        this.RightLift = RightLift;
        this.Elbow = Elbow;
        this.SlideEncoder = slideEncoder;
    }

    public void pause() {
        int target = Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition());
        LeftLift.setTargetPosition(target);
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setPower(pidPower);
        RightLift.setTargetPosition(target);
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightLift.setPower(pidPower);
        liftPower = pidPower;
        Elbow.setPosition(0.5);
    }

    public void stop() {
        LeftLift.setPower(0);
        RightLift.setPower(0);
    }

    public void start() {
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int LiftTarget = (int) Math.round(liftTargetIN * LiftTicksPerInch);
        LeftLift.setTargetPosition(LiftTarget);
        RightLift.setTargetPosition(LiftTarget);
    }

    public void start(double liftTargetIN) {
        RightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftTargetIN = liftTargetIN;
        int LiftTarget = (int) Math.round(liftTargetIN * LiftTicksPerInch);
        LeftLift.setTargetPosition(LiftTarget);
        RightLift.setTargetPosition(LiftTarget);
        LeftLift.setPower(pidPower);
        RightLift.setPower(pidPower);
    }

    public void update(RevBulkData bulkData2) {


        LiftPositionIN = Math.max(LeftLift.getCurrentPosition(), RightLift.getCurrentPosition()) / LiftTicksPerInch;
        SlidePositionIN = bulkData2.getMotorCurrentPosition(SlideEncoder) / SlideTicksPerInch;

        boolean liftObstruction = SlidePositionIN < 6 && SlidePositionIN > 1;
        boolean slideObstruction = false;

        if (liftObstruction) {
            if (liftTargetIN < 6 && LiftPositionIN < 6) {
                int LiftTarget = (int) Math.round(7 * LiftTicksPerInch);
                LeftLift.setTargetPosition(LiftTarget);
                RightLift.setTargetPosition(LiftTarget);
            } else if (LiftPositionIN < 6) {
                slideObstruction = true;
            }
        } else {
            int LiftTarget = (int) Math.round(liftTargetIN * LiftTicksPerInch);
            LeftLift.setTargetPosition(LiftTarget);
            RightLift.setTargetPosition(LiftTarget);
        }


        double liftOffset = (bulkData2.getMotorCurrentPosition(LeftLift) - bulkData2.getMotorCurrentPosition(RightLift)) / (LeftLift.getMotorType().getTicksPerRev());

        LeftLift.setPower(liftPower + Math.max(0, -liftOffset));
        RightLift.setPower(liftPower + Math.max(0, liftOffset));

        if (Math.abs(LiftPositionIN - liftTargetIN) < tolerance) {
            isBusy = false;
            LeftLift.setPower(pidPower);
            RightLift.setPower(pidPower);
            liftPower = pidPower;
        } else
            isBusy = true;

        if (Math.abs(SlidePositionIN - slideTargetIN) < .15 || slideObstruction) {
            Elbow.setPosition(0.5);
        } else {
            Elbow.setPosition(Range.clip(
                    Range.scale(
                            (slideTargetIN - SlidePositionIN), -1, 1, 0.2, 0.8),
                    0.2, 0.8));
            isBusy = true;
        }

    }
}
