package pid

import (
	"math"
	"time"
)

// AntiWindupController implements a PID-controller with low-pass filter of the derivative term,
// feed forward term, a saturated control output and anti-windup.
//
// The anti-windup mechanism uses an actuator saturation model as defined in Chapter 6 of Åström and Murray,
// Feedback Systems: An Introduction to Scientists and Engineers, 2008
// (http://www.cds.caltech.edu/~murray/amwiki)
//
// The ControlError, ControlErrorIntegrand, ControlErrorIntegral and ControlErrorDerivative are prevented
// from reaching +/- inf by clamping them to [-math.MaxFloat64, math.MaxFloat64].
type AntiWindupController struct {
	// Config for the AntiWindupController.
	Config AntiWindupControllerConfig
	// State of the AntiWindupController.
	State AntiWindupControllerState
}

// AntiWindupControllerConfig contains config parameters for a AntiWindupController.
type AntiWindupControllerConfig struct {
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

// AntiWindupControllerState holds mutable state for a AntiWindupController.
type AntiWindupControllerState struct {
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
	// UnsaturatedControlSignal is the control signal before saturation.
	UnsaturatedControlSignal float64
}

// AntiWindupControllerInput holds the input parameters to an AntiWindupController.
type AntiWindupControllerInput struct {
	// ReferenceSignal is the reference value for the signal to control.
	ReferenceSignal float64
	// ActualSignal is the actual value of the signal to control.
	ActualSignal float64
	// FeedForwardSignal is the contribution of the feed-forward control loop in the controller output.
	FeedForwardSignal float64
	// SamplingInterval is the time interval elapsed since the previous call of the controller Update method.
	SamplingInterval time.Duration
}

// Reset the controller state.
func (c *AntiWindupController) Reset() {
	c.State = AntiWindupControllerState{}
}

// Update the controller state.
func (c *AntiWindupController) Update(input AntiWindupControllerInput) {

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
	c.State.ControlErrorIntegrand = e + c.Config.AntiWindUpGain*(c.State.ControlSignal-c.State.UnsaturatedControlSignal)
	c.State.ControlErrorIntegrand = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, c.State.ControlErrorIntegrand))
	c.State.ControlErrorIntegral = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, controlErrorIntegral))
	c.State.ControlErrorDerivative = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, controlErrorDerivative))
	c.State.ControlError = math.Max(-math.MaxFloat64, math.Min(math.MaxFloat64, e))
}

// DischargeIntegral provides the ability to discharge the controller integral state
// over a configurable period of time.
func (c *AntiWindupController) DischargeIntegral(dt time.Duration) {
	c.State.ControlErrorIntegrand = 0.0
	c.State.ControlErrorIntegral = math.Max(
		0,
		math.Min(1-dt.Seconds()/c.Config.IntegralDischargeTimeConstant, 1.0),
	) * c.State.ControlErrorIntegral
}
