package org.firstinspires.ftc.teamcode.common.util;

public class Transform3d {
    private final Translation3d m_translation;
    private final Rotation3d m_rotation;

    public Transform3d(Pose3d initial, Pose3d last) {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        m_translation =
                last.getTranslation()
                        .minus(initial.getTranslation())
                        .rotateBy(initial.getRotation().unaryMinus());

        m_rotation = last.getRotation().minus(initial.getRotation());
    }

    public Transform3d(Translation3d translation, Rotation3d rotation) {
        m_translation = translation;
        m_rotation = rotation;
    }

    public Transform3d(double x, double y, double z, Rotation3d rotation) {
        m_translation = new Translation3d(x, y, z);
        m_rotation = rotation;
    }

    public Transform3d() {
        m_translation = new Translation3d();
        m_rotation = new Rotation3d();
    }

    public Translation3d getTranslation() {
        return m_translation;
    }

    public Rotation3d getRotation() {
        return m_rotation;
    }

    public Transform3d inverse() {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        return new Transform3d(
                getTranslation().unaryMinus().rotateBy(getRotation().unaryMinus()),
                getRotation().unaryMinus());
    }




}
