package pid

import "time"

// Controller implements a basic PID controller.
type Controller struct {
	// Config for the Controller.
	Config ControllerConfig
	// State of the Controller.
	State ControllerState
}

// ControllerConfig contains configurable parameters for a Controller.
type ControllerConfig struct {
	// ProportionalGain determines ratio of output response to error signal.
	ProportionalGain float64
	// IntegralGain determines previous error's affect on output.
	IntegralGain float64
	// DerivativeGain decreases the sensitivity to large reference changes.
	DerivativeGain float64
}

// ControllerState holds mutable state for a Controller.
type ControllerState struct {
	// ControlError is the difference between reference and current value.
	ControlError float64
	// ControlErrorIntegral is the integrated control error over time.
	ControlErrorIntegral float64
	// ControlErrorDerivative is the rate of change of the control error.
	ControlErrorDerivative float64
	// ControlSignal is the current control signal output of the controller.
	ControlSignal float64
}

// ControllerInput holds the input parameters to a Controller.
type ControllerInput struct {
	// ReferenceSignal is the reference value for the signal to control.
	ReferenceSignal float64
	// ActualSignal is the actual value of the signal to control.
	ActualSignal float64
	// SamplingInterval is the time interval elapsed since the previous call of the controller Update method.
	SamplingInterval time.Duration
}

// Update the controller state.
func (c *Controller) Update(input ControllerInput) {
	previousError := c.State.ControlError
	c.State.ControlError = input.ReferenceSignal - input.ActualSignal
	c.State.ControlErrorDerivative = (c.State.ControlError - previousError) / input.SamplingInterval.Seconds()
	c.State.ControlErrorIntegral += c.State.ControlError * input.SamplingInterval.Seconds()
	c.State.ControlSignal =
		c.Config.ProportionalGain*c.State.ControlError +
			c.Config.IntegralGain*c.State.ControlErrorIntegral +
			c.Config.DerivativeGain*c.State.ControlErrorDerivative
}

// Reset the controller state.
func (c *Controller) Reset() {
	c.State = ControllerState{}
}
