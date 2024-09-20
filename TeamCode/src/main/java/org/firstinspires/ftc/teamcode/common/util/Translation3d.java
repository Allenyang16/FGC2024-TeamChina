package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.Objects;

public class Translation3d {
    private final double m_x;
    private final double m_y;
    private final double m_z;

    public Translation3d() {
        this(0.0, 0.0, 0.0);
    }

    public Translation3d(double x, double y, double z) {
        m_x = x;
        m_y = y;
        m_z = z;
    }

    public double getNorm() {
        return Math.sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
    }

    public double getDistance(Translation3d other) {
        return Math.sqrt(
                Math.pow(other.m_x - m_x, 2) + Math.pow(other.m_y - m_y, 2) + Math.pow(other.m_z - m_z, 2));
    }

    public Translation3d rotateBy(Rotation3d other) {
        final Quaternion p = new Quaternion(0.0, m_x, m_y, m_z);
        final Quaternion qprime = other.getQuaternion().times(p).times(other.getQuaternion().inverse());
        return new Translation3d(qprime.getX(), qprime.getY(), qprime.getZ());
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(m_x, m_y);
    }

    public Translation3d plus(Translation3d other) {
        return new Translation3d(m_x + other.m_x, m_y + other.m_y, m_z + other.m_z);
    }

    public Translation3d minus(Translation3d other) {
        return new Translation3d(m_x - other.m_x, m_y - other.m_y, m_z - other.m_z);
    }

    public Translation3d unaryMinus() {
        return new Translation3d(-m_x, -m_y, -m_z);
    }

    public Translation3d times(double scalar) {
        return new Translation3d(m_x * scalar, m_y * scalar, m_z * scalar);
    }

    public Translation3d div(double scalar) {
        return new Translation3d(m_x / scalar, m_y / scalar, m_z / scalar);
    }

    @Override
    public String toString() {
        return String.format("Translation3d(X: %.2f, Y: %.2f, Z: %.2f)", m_x, m_y, m_z);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj instanceof Translation3d) {
            return Math.abs(((Translation3d) obj).m_x - m_x) < 1E-9
                    && Math.abs(((Translation3d) obj).m_y - m_y) < 1E-9
                    && Math.abs(((Translation3d) obj).m_z - m_z) < 1E-9;
        }
        return false;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_x, m_y, m_z);
    }

}
