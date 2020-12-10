package pid

import (
	"math"
	"time"
)

// TrackingController implements a PIDT1 controller with feed forward term, anti-windup and bumpless
// transfer using tracking mode control.
//
// The DT1-part behaves much like a D-part up to a tunable cut-off frequency.
//
// The anti-windup and bumpless transfer mechanisms use a tracking mode as defined in
// Chapter 6 of Åström and Murray, Feedback Systems:
// An Introduction to Scientists and Engineers, 2008
// (https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
type TrackingController struct {
	// Config for the TrackingController.
	Config TrackingControllerConfig
	// State of the TrackingController.
	State TrackingControllerState
}

// TrackingControllerConfig contains configurable parameters for a TrackingController.
type TrackingControllerConfig struct {
	// ProportionalGain is the P part gain.
	ProportionalGain float64
	// IntegralGain is the I part gain.
	IntegralGain float64
	// DerivativeGain is the D part gain.
	DerivativeGain float64
	// AntiWindUpGain is the anti-windup tracking gain.
	AntiWindUpGain float64
	// Inverse of the time constant to discharge the integral part of the PID controller (GO-button released) [1/s]
	IntegralPartDecreaseFactor float64
	// LowPassTimeConstant is the D part low-pass filter time constant => cut-off frequency 1/LowPassTimeConstant.
	LowPassTimeConstant time.Duration
	// MaxOutput is the max output from the PID.
	MaxOutput float64
	// MinOutput is the min output from the PID.
	MinOutput float64
}

// TrackingControllerState holds the mutable state a TrackingController.
type TrackingControllerState struct {
	// ControlError is the difference between reference and current value.
	ControlError float64
	// ControlErrorIntegral is the integrated control error over time.
	ControlErrorIntegral float64
	// TODO: Document me.
	IntegralState float64
	// TODO: Document me.
	DerivativeState float64
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float64
}

// Reset the controller state.
func (c *TrackingController) Reset() {
	c.State = TrackingControllerState{}
}

// Update the controller state.
func (c *TrackingController) Update(
	target float64,
	actual float64,
	ff float64,
	actualInput float64,
	dt time.Duration,
) float64 {
	e := target - actual
	uP := e * c.Config.ProportionalGain
	uI := c.State.ControlErrorIntegral*c.Config.IntegralGain*dt.Seconds() + c.State.IntegralState
	uD := ((c.Config.DerivativeGain/c.Config.LowPassTimeConstant.Seconds())*
		(e-c.State.ControlError) + c.State.DerivativeState) /
		(dt.Seconds()/c.Config.LowPassTimeConstant.Seconds() + 1)
	uV := uP + uI + uD + ff
	c.State.ControlErrorIntegral = e + c.Config.AntiWindUpGain*(actualInput-c.State.ControlSignal)
	c.State.IntegralState = uI
	c.State.DerivativeState = uD
	c.State.ControlSignal = uV
	c.State.ControlError = e
	return math.Max(c.Config.MinOutput, math.Min(c.Config.MaxOutput, uV))
}

// TODO: Document me.
func (c *TrackingController) DischargeIntegral(dt time.Duration) {
	c.State.ControlErrorIntegral = 0.0
	c.State.IntegralState = math.Max(
		0,
		math.Min(1-dt.Seconds()*c.Config.IntegralPartDecreaseFactor, 1.0),
	) * c.State.IntegralState
}
