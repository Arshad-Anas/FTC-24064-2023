package org.firstinspires.ftc.teamcode.control.gainmatrices

import kotlin.math.*

data class PIDGains

@JvmOverloads
constructor(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var maxOutputWithIntegral: Double = Double.POSITIVE_INFINITY,
) {
    @JvmOverloads
    fun computeKd(gains: FeedforwardGains, percentOver: Double = 0.0): PIDGains {
        val overshoot = percentOver / 100.0
        val zeta: Double = if (overshoot <= 0.0) 1.0 else -ln(overshoot) / sqrt(PI.pow(2) + ln(overshoot).pow(2))
        kD = max(
                0.0,
                2 * zeta * sqrt(gains.kA * kP) - gains.kV
        )
        return this
    }
}
