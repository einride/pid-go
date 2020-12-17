package pid

import (
	"math"
	"time"
)

// SaturatedController implements a PID-controller with low-pass filter of the derivative term,
// feed forward term, a saturated control output and anti-windup.
//
// The anti-windup mechanism uses an actuator saturation model as defined in Chapter 6 of Åström and Murray,
// Feedback Systems: An Introduction to Scientists and Engineers, 2008
// (http://www.cds.caltech.edu/~murray/amwiki)
type SaturatedController struct {
	// Config for the SaturatedController.
	Config SaturatedControllerConfig
	// State of the SaturatedController.
	State SaturatedControllerState
}

// SaturatedControllerConfig contains config parameters for a SaturatedController.
type SaturatedControllerConfig struct {
	// ProportionalGain is the P part gain.
	ProportionalGain float64
	// IntegralGain is the I part gain.
	IntegralGain float64
	// DerivativeGain is the D part gain.
	DerivativeGain float64
	// AntiWindUpGain is the anti-windup tracking gain.
	AntiWindUpGain float64
	// Inverse of the time constant to discharge the integral part of the PID controller (1/s)
	IntegralPartDischargeFactor float64
	// LowPassTimeConstant is the D part low-pass filter time constant => cut-off frequency 1/LowPassTimeConstant.
	LowPassTimeConstant time.Duration
	// MaxOutput is the max output from the PID.
	MaxOutput float64
	// MinOutput is the min output from the PID.
	MinOutput float64
}

// SaturatedControllerState holds mutable state for a SaturatedController.
type SaturatedControllerState struct {
	// ControlError is the difference between reference and current value.
	ControlError float64
	// ControlErrorIntegrand is the control error integrand, which includes the anti-windup correction.
	ControlErrorIntegrand float64
	// ControlErrorIntegral is the control error integrand integrated over time.
	ControlErrorIntegral float64
	// ControlErrorDerivative is the low-pass filtered time-derivative of the control error.
	ControlErrorDerivative float64
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float64
}

// Reset the controller state.
func (c *SaturatedController) Reset() {
	c.State = SaturatedControllerState{}
}

// Update the controller state.
func (c *SaturatedController) Update(
	referenceSignal float64,
	actualSignal float64,
	feedForwardSignal float64,
	dt time.Duration,
) {
	e := referenceSignal - actualSignal
	controlErrorIntegral := c.State.ControlErrorIntegrand*dt.Seconds() + c.State.ControlErrorIntegral
	controlErrorDerivative := ((1/c.Config.LowPassTimeConstant.Seconds())*(e-c.State.ControlError) +
		c.State.ControlErrorDerivative) / (dt.Seconds()/c.Config.LowPassTimeConstant.Seconds() + 1)
	u := e*c.Config.ProportionalGain + c.Config.IntegralGain*controlErrorIntegral +
		c.Config.DerivativeGain*controlErrorDerivative + feedForwardSignal
	c.State.ControlSignal = math.Max(c.Config.MinOutput, math.Min(c.Config.MaxOutput, u))
	c.State.ControlErrorIntegrand = e + c.Config.AntiWindUpGain*(c.State.ControlSignal-u)
	c.State.ControlErrorIntegral = controlErrorIntegral
	c.State.ControlErrorDerivative = controlErrorDerivative
	c.State.ControlError = e
}

// TODO: Document me.
func (c *SaturatedController) DischargeIntegral(dt time.Duration) {
	c.State.ControlErrorIntegrand = 0.0
	c.State.ControlErrorIntegral = math.Max(
		0,
		math.Min(1-dt.Seconds()*c.Config.IntegralPartDischargeFactor, 1.0),
	) * c.State.ControlErrorIntegral
}
