package pid

import (
	"math"
	"time"
)

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
	// MinOutput sets an lower limit to the allowed output.
	MinOutput float64
	// MaxOutput sets an upper limit to the allowed output.
	MaxOutput float64
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

// Update the controller state.
func (s *Controller) Update(referenceSignal float64, actualSignal float64, dt time.Duration) float64 {
	previousError := s.State.ControlError
	s.State.ControlError = referenceSignal - actualSignal
	s.State.ControlErrorDerivative = (s.State.ControlError - previousError) / dt.Seconds()
	s.State.ControlErrorIntegral += s.State.ControlError * dt.Seconds()
	s.State.ControlSignal = math.Max(s.Config.MinOutput, math.Min(s.Config.MaxOutput, s.output()))
	return s.State.ControlSignal
}

// Reset the controller state.
func (s *Controller) Reset() {
	s.State = ControllerState{}
}

// output calculates the necessary output to maintain or reach a reference.
func (s *Controller) output() float64 {
	return s.Config.ProportionalGain*s.State.ControlError +
		s.Config.IntegralGain*s.State.ControlErrorIntegral +
		s.Config.DerivativeGain*s.State.ControlErrorDerivative
}
