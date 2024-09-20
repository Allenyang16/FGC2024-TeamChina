package org.firstinspires.ftc.teamcode.common.util;

public class Quaternion {
    private final double m_w;

    private final double m_x;
    private final double m_y;
    private final double m_z;

    public Quaternion() {
        m_w = 1.0;
        m_x = 0.0;
        m_y = 0.0;
        m_z = 0.0;
    }

    public Quaternion(double w, double x, double y, double z) {
        m_w = w;
        m_x = x;
        m_y = y;
        m_z = z;
    }

    public Quaternion plus(Quaternion other) {
        return new Quaternion(
                getW() + other.getW(), getX() + other.getX(), getY() + other.getY(), getZ() + other.getZ());
    }

    public Quaternion minus(Quaternion other) {
        return new Quaternion(
                getW() - other.getW(), getX() - other.getX(), getY() - other.getY(), getZ() - other.getZ());
    }

    public Quaternion divide(double scalar) {
        return new Quaternion(getW() / scalar, getX() / scalar, getY() / scalar, getZ() / scalar);
    }

    public Quaternion times(double scalar) {
        return new Quaternion(getW() * scalar, getX() * scalar, getY() * scalar, getZ() * scalar);
    }

    public Quaternion times(Quaternion other) {
        // https://en.wikipedia.org/wiki/Quaternion#Scalar_and_vector_parts
        final double r1 = m_w;
        final double r2 = other.m_w;

        // v₁ ⋅ v₂
        double dot = m_x * other.m_x + m_y * other.m_y + m_z * other.m_z;

        // v₁ x v₂
        double cross_x = m_y * other.m_z - other.m_y * m_z;
        double cross_y = other.m_x * m_z - m_x * other.m_z;
        double cross_z = m_x * other.m_y - other.m_x * m_y;

        return new Quaternion(
                // r = r₁r₂ − v₁ ⋅ v₂
                r1 * r2 - dot,
                // v = r₁v₂ + r₂v₁ + v₁ x v₂
                r1 * other.m_x + r2 * m_x + cross_x,
                r1 * other.m_y + r2 * m_y + cross_y,
                r1 * other.m_z + r2 * m_z + cross_z);
    }

    public Quaternion conjugate() {
        return new Quaternion(getW(), -getX(), -getY(), -getZ());
    }

    public double dot(final Quaternion other) {
        return getW() * other.getW()
                + getX() * other.getX()
                + getY() * other.getY()
                + getZ() * other.getZ();
    }

    public Quaternion inverse() {
        double norm = norm();
        return conjugate().divide(norm * norm);
    }

    public double norm() {
        return Math.sqrt(dot(this));
    }

    public Quaternion normalize() {
        double norm = norm();
        if (norm == 0.0) {
            return new Quaternion();
        } else {
            return new Quaternion(getW() / norm, getX() / norm, getY() / norm, getZ() / norm);
        }
    }


    public double getW() {
        return m_w;
    }

    public double getX() {
        return m_x;
    }

    public double getY() {
        return m_y;
    }

    public double getZ() {
        return m_z;
    }


}
