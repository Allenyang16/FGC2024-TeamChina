package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class Pose3d {
    private final Translation3d m_translation;
    private final Rotation3d m_rotation;

    public Pose3d() {
        m_translation = new Translation3d();
        m_rotation = new Rotation3d();
    }

    public Pose3d(Translation3d translation, Rotation3d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    public Pose3d(double x, double y, double z, Rotation3d rotation) {
        m_translation = new Translation3d(x, y, z);
        m_rotation = rotation;
    }

    public Pose3d(Pose2d pose) {
        m_translation = new Translation3d(pose.getX(), pose.getY(), 0.0);
        m_rotation = new Rotation3d(0.0, 0.0, pose.getRotation().getRadians());
    }

    public Pose3d rotateBy(Rotation3d other) {
        return new Pose3d(m_translation.rotateBy(other), m_rotation.rotateBy(other));
    }

    public Translation3d getTranslation() {
        return m_translation;
    }

    public Rotation3d getRotation() {
        return m_rotation;
    }

    public Pose3d transformBy(Transform3d other) {
        return new Pose3d(
                m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
                other.getRotation().plus(m_rotation));
    }

    public Pose3d relativeTo(Pose3d other) {
        Transform3d transform = new Transform3d(other, this);
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    public Pose2d toPose2d() {
        return new Pose2d(m_translation.toTranslation2d(), m_rotation.toRotation2d());
    }

    public Transform3d toTransform3d() {
        return new Transform3d(m_translation, m_rotation);
    }
}
