// Package speed implements a PID controller for reaching and maintaining a reference value.
package pid

import (
	"math"
	"time"
)

type SimpleController struct {
	errorOverTime float64 // errorOverTime is the integral error (sum of errors).
	errorRate     float64 // errorRate is the derivative error (prop. to rate of change in reference).
	errorSize     float64 // error is the difference between reference and current value.

	ProportionalGain float64 // ProportionalGain determines ratio of output response to error signal.
	IntegrationGain  float64 // IntegrationGain determines previous error's affect on output.
	DerivationGain   float64 // DerivationGain decreases the sensitivity to large reference changes.
	OutputLowerLimit float64 // OutputLowerLimit sets an lower limit to the allowed output.
	OutputUpperLimit float64 // OutputUpperLimit sets an upper limit to the allowed output.
}

func (s *SimpleController) Update(reference, current float64, dt time.Duration) float64 {
	previousError := s.errorSize
	s.errorSize = reference - current
	s.errorRate = (s.errorSize - previousError) / dt.Seconds()
	s.errorOverTime += s.errorSize * dt.Seconds()
	return math.Max(s.OutputLowerLimit, math.Min(s.OutputUpperLimit, s.calculatePIDOutput()))
}

func (s *SimpleController) Reset() {
	s.errorOverTime = 0
	s.errorRate = 0
	s.errorSize = 0
}

// calculatePIDOutput calculates the necessary output to maintain or reach a reference.
func (s *SimpleController) calculatePIDOutput() (outputValue float64) {
	return s.ProportionalGain*s.errorSize + s.DerivationGain*s.errorRate + s.IntegrationGain*s.errorOverTime
}
