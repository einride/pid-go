package pid

import "time"

// Controller implements a basic PID controller.
type Controller struct {
	Config ControllerConfig
	State  ControllerState
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
	// TODO: Document me.
	ReferenceSignal float64
	// TODO: Document me.
	ActualSignal float64
	// TODO: Document me.
	SamplingInterval time.Duration
}

// Update the controller state.
func (s *Controller) Update(input ControllerInput) float64 {
	previousError := s.State.ControlError
	s.State.ControlError = input.ReferenceSignal - input.ActualSignal
	s.State.ControlErrorDerivative = (s.State.ControlError - previousError) / input.SamplingInterval.Seconds()
	s.State.ControlErrorIntegral += s.State.ControlError * input.SamplingInterval.Seconds()
	s.State.ControlSignal = s.Config.ProportionalGain*s.State.ControlError +
		s.Config.IntegralGain*s.State.ControlErrorIntegral +
		s.Config.DerivativeGain*s.State.ControlErrorDerivative
	return s.State.ControlSignal
}

// Reset the controller state.
func (s *Controller) Reset() {
	s.State = ControllerState{}
}
