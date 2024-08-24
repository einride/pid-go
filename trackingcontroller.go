package pid

import (
	"math"
	"time"
)

// TrackingController implements a PID-controller with low-pass filter of the derivative term,
// feed forward term, anti-windup and bumpless transfer using tracking mode control.
//
// The anti-windup and bumpless transfer mechanisms use a tracking mode as defined in
// Chapter 6 of Åström and Murray, Feedback Systems:
// An Introduction to Scientists and Engineers, 2008
// (http://www.cds.caltech.edu/~murray/amwiki)
//
// The ControlError, ControlErrorIntegrand, ControlErrorIntegral and ControlErrorDerivative are prevented
// from reaching +/- inf by clamping them to [-math.MaxFloat64, math.MaxFloat64].
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
	// IntegralDischargeTimeConstant is the time constant to discharge the integral state of the PID controller (s)
	IntegralDischargeTimeConstant float64
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
	// ControlErrorIntegrand is the integrated control error over time.
	ControlErrorIntegrand float64
	// ControlErrorIntegral is the control error integrand integrated over time.
	ControlErrorIntegral float64
	// ControlErrorDerivative is the low-pass filtered time-derivative of the control error.
	ControlErrorDerivative float64
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float64
	// UnsaturatedControlSignal is the control signal before saturation used for tracking the
	// actual control signal for bumpless transfer or compensation of un-modeled saturations.
	UnsaturatedControlSignal float64
}

// TrackingControllerInput holds the input parameters to a TrackingController.
type TrackingControllerInput struct {
	// ReferenceSignal is the reference value for the signal to control.
	ReferenceSignal float64
	// ActualSignal is the actual value of the signal to control.
	ActualSignal float64
	// FeedForwardSignal is the contribution of the feed-forward control loop in the controller output.
	FeedForwardSignal float64
	// AppliedControlSignal is the actual control command applied by the actuator.
	AppliedControlSignal float64
	// SamplingInterval is the time interval elapsed since the previous call of the controller Update method.
	SamplingInterval time.Duration
}

// Reset the controller state.
func (c *TrackingController) Reset() {
	c.State = TrackingControllerState{}
}

// Update the controller state.
func (c *TrackingController) Update(input TrackingControllerInput) {
	if math.IsNaN(input.ReferenceSignal) || math.IsNaN(input.ActualSignal) ||
		math.IsInf(input.ReferenceSignal, 0) || math.IsInf(input.ActualSignal, 0) {
		return
	}
	e := input.ReferenceSignal - input.ActualSignal
	controlErrorIntegral := c.State.ControlErrorIntegrand*input.SamplingInterval.Seconds() + c.State.ControlErrorIntegral
	controlErrorDerivative := ((1/c.Config.LowPassTimeConstant.Seconds())*(e-c.State.ControlError) +
		c.State.ControlErrorDerivative) / (input.SamplingInterval.Seconds()/c.Config.LowPassTimeConstant.Seconds() + 1)
	c.State.UnsaturatedControlSignal = e*c.Config.ProportionalGain + c.Config.IntegralGain*controlErrorIntegral +
		c.Config.DerivativeGain*controlErrorDerivative + input.FeedForwardSignal
	c.State.ControlSignal = math.Max(c.Config.MinOutput, math.Min(c.Config.MaxOutput, c.State.UnsaturatedControlSignal))
	c.State.ControlErrorIntegrand = e + c.Config.AntiWindUpGain*(input.AppliedControlSignal-
		c.State.UnsaturatedControlSignal)
	c.State.ControlErrorIntegrand = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, c.State.ControlErrorIntegrand))
	c.State.ControlErrorIntegral = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, controlErrorIntegral))
	c.State.ControlErrorDerivative = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, controlErrorDerivative))
	c.State.ControlError = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, e))
}

// DischargeIntegral provides the ability to discharge the controller integral state
// over a configurable period of time.
func (c *TrackingController) DischargeIntegral(dt time.Duration) {
	c.State.ControlErrorIntegrand = 0.0
	c.State.ControlErrorIntegral = math.Max(
		0,
		math.Min(1-dt.Seconds()/c.Config.IntegralDischargeTimeConstant, 1.0),
	) * c.State.ControlErrorIntegral
}
