package pid

import (
	"math"
	"time"

	adv1 "github.com/einride/proto/gen/go/ad/v1"
	"google.golang.org/protobuf/types/known/timestamppb"
)

type SimpleController struct {
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

	state simpleState
}

type simpleState struct {
	// errorOverTime is the integral error (sum of errors).
	errorOverTime float64
	// errorRate is the derivative error (prop. to rate of change in reference).
	errorRate float64
	// error is the difference between reference and current value.
	errorSize float64
}

func (s *SimpleController) Update(reference, current float64, dt time.Duration) float64 {
	previousError := s.state.errorSize
	s.state.errorSize = reference - current
	s.state.errorRate = (s.state.errorSize - previousError) / dt.Seconds()
	s.state.errorOverTime += s.state.errorSize * dt.Seconds()
	return math.Max(s.MinOutput, math.Min(s.MaxOutput, s.output()))
}

func (s *SimpleController) Reset() {
	s.state = simpleState{}
}

// output calculates the necessary output to maintain or reach a reference.
func (s *SimpleController) output() float64 {
	return s.ProportionalGain*s.state.errorSize +
		s.IntegralGain*s.state.errorOverTime +
		s.DerivativeGain*s.state.errorRate
}

func (s *SimpleController) GetState(now time.Time) *adv1.PIDState {
	return &adv1.PIDState{
		Time:            &timestamppb.Timestamp{Seconds: now.Unix(), Nanos: int32(now.Nanosecond())},
		Error:           float32(s.state.errorSize),
		IntegralError:   float32(s.state.errorOverTime),
		DerivativeState: float32(s.state.errorOverTime),
	}
}
