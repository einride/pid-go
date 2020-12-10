package pid

import (
	"math"
	"time"
)

type Controller struct {
	Config ControllerConfig
	State  ControllerState
}

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

type ControllerState struct {
	// ErrorOverTime is the integral error (sum of errors).
	ErrorOverTime float64
	// ErrorRate is the derivative error (prop. to rate of change in reference).
	ErrorRate float64
	// ErrorSize is the difference between reference and current value.
	ErrorSize float64
}

func (s *Controller) Update(reference float64, current float64, dt time.Duration) float64 {
	previousError := s.State.ErrorSize
	s.State.ErrorSize = reference - current
	s.State.ErrorRate = (s.State.ErrorSize - previousError) / dt.Seconds()
	s.State.ErrorOverTime += s.State.ErrorSize * dt.Seconds()
	return math.Max(s.Config.MinOutput, math.Min(s.Config.MaxOutput, s.output()))
}

func (s *Controller) Reset() {
	s.State = ControllerState{}
}

// output calculates the necessary output to maintain or reach a reference.
func (s *Controller) output() float64 {
	return s.Config.ProportionalGain*s.State.ErrorSize +
		s.Config.IntegralGain*s.State.ErrorOverTime +
		s.Config.DerivativeGain*s.State.ErrorRate
}
