package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class Rotation3d {
    private final Quaternion m_q;

    public Rotation3d() {
        m_q = new Quaternion();
    }

    public Rotation3d(Quaternion q) {
        m_q = q.normalize();
    }

    public Rotation3d(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);

        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);

        m_q =
                new Quaternion(
                        (cr * cp * cy + sr * sp * sy),
                        (sr * cp * cy - cr * sp * sy),
                        (cr * sp * cy + sr * cp * sy),
                        (cr * cp * sy - sr * sp * cy));
    }

    public Rotation3d(VectorF axis, double angleRadians) {
        double norm = axis.magnitude();
        if (norm == 0.0) {
            m_q = new Quaternion();
            return;
        }

        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Definition
        VectorF v = axis.multiplied((float) (1.0 / norm)).multiplied((float) Math.sin(angleRadians / 2.0));
        m_q = new Quaternion(Math.cos(angleRadians / 2.0), v.get(0), v.get(1), v.get(2));
    }

    public Quaternion getQuaternion() {
        return m_q;
    }

    public Rotation3d rotateBy(Rotation3d other) {
        return new Rotation3d(other.m_q.times(m_q));
    }

    public Rotation3d plus(Rotation3d other) {
        return rotateBy(other);
    }

    public Rotation3d minus(Rotation3d other) {
        return rotateBy(other.unaryMinus());
    }

    public Rotation3d unaryMinus() {
        return new Rotation3d(m_q.inverse());
    }

    public double getX() {
        final double w = m_q.getW();
        final double x = m_q.getX();
        final double y = m_q.getY();
        final double z = m_q.getZ();

        // wpimath/algorithms.md
        final double cxcy = 1.0 - 2.0 * (x * x + y * y);
        final double sxcy = 2.0 * (w * x + y * z);
        final double cy_sq = cxcy * cxcy + sxcy * sxcy;
        if (cy_sq > 1e-20) {
            return Math.atan2(sxcy, cxcy);
        } else {
            return 0.0;
        }
    }

    /**
     * Returns the counterclockwise rotation angle around the Y axis (pitch) in radians.
     *
     * @return The counterclockwise rotation angle around the Y axis (pitch) in radians.
     */
    public double getY() {
        final double w = m_q.getW();
        final double x = m_q.getX();
        final double y = m_q.getY();
        final double z = m_q.getZ();

        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
        double ratio = 2.0 * (w * y - z * x);
        if (Math.abs(ratio) >= 1.0) {
            return Math.copySign(Math.PI / 2.0, ratio);
        } else {
            return Math.asin(ratio);
        }
    }

    /**
     * Returns the counterclockwise rotation angle around the Z axis (yaw) in radians.
     *
     * @return The counterclockwise rotation angle around the Z axis (yaw) in radians.
     */
    public double getZ() {
        final double w = m_q.getW();
        final double x = m_q.getX();
        final double y = m_q.getY();
        final double z = m_q.getZ();

        // wpimath/algorithms.md
        final double cycz = 1.0 - 2.0 * (y * y + z * z);
        final double cysz = 2.0 * (w * z + x * y);
        final double cy_sq = cycz * cycz + cysz * cysz;
        if (cy_sq > 1e-20) {
            return Math.atan2(cysz, cycz);
        } else {
            return Math.atan2(2.0 * w * z, w * w - z * z);
        }
    }

    public Rotation2d toRotation2d() {
        return new Rotation2d(getZ());
    }

}
