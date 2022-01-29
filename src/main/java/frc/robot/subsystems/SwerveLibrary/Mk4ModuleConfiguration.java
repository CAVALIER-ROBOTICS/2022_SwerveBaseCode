// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveLibrary;

import java.util.Objects;

/** Add your docs here. */
public class Mk4ModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 80.0;
    private double steerCurrentLimit = 20.0;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Mk4ModuleConfiguration that = (Mk4ModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "Mk4ModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                '}';
    }
}

