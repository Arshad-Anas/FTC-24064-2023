package org.firstinspires.ftc.teamcode.subsystems.centerstage.auton;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.isTop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class EditablePose {
    public double x, y, heading;

    public EditablePose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    // Switches *red* alliance coordinates to blue alliance
    public EditablePose byAlliance() {
        EditablePose pose = new EditablePose(x, y, heading);
        if (!isRed) {
            pose.y *= -1;
            pose.heading *= -1;
        }
        return pose;
    }

    public Pose2d byAlliancePose2d() {
        return byAlliance().toPose2d();
    }

    public Vector2d byAllianceVec() {
        return byAlliancePose2d().vec();
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, heading);
    }

    // Switches audience-side coordinates to backboard-side
    public EditablePose bySide() {
        EditablePose pose = new EditablePose(x, y, heading);
        if (isTop) pose.x = -x - 23.5;
        return pose;
    }

    public Pose2d bySidePose2d() {
        return bySide().toPose2d();
    }

    public Vector2d bySideVec() {
        return bySidePose2d().vec();
    }

    public EditablePose byBoth() {
        return bySide().byAlliance();
    }

    public Pose2d byBothPose2d() {
        return byBoth().toPose2d();
    }

    public Vector2d byBothVec() {
        return byBothPose2d().vec();
    }
}
