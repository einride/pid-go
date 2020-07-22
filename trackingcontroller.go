package pid

import (
	"math"
	"time"

	adv1 "github.com/einride/proto/gen/go/ad/v1"
	"google.golang.org/protobuf/types/known/timestamppb"
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
//
// The controller structure is based on Steer-by-Wire Controller Development for Einride's T-Pod:
// Phase One, Michele Sigilló, 2018.
type TrackingController struct {
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

	state trackingState
}

type trackingState struct {
	e  float64
	eI float64
	uI float64
	uD float64
	uV float64
}

func (c *TrackingController) Reset() {
	c.state = trackingState{}
}

func (c *TrackingController) Update(
	target float64,
	actual float64,
	ff float64,
	actualInput float64,
	dt time.Duration,
) float64 {
	e := target - actual
	uP := e * c.ProportionalGain
	uI := c.state.eI*c.IntegralGain*dt.Seconds() + c.state.uI
	uD := ((c.DerivativeGain/c.LowPassTimeConstant.Seconds())*(e-c.state.e) + c.state.uD) /
		(dt.Seconds()/c.LowPassTimeConstant.Seconds() + 1)
	uV := uP + uI + uD + ff
	c.state.eI = e + c.AntiWindUpGain*(actualInput-c.state.uV)
	c.state.uI = uI
	c.state.uD = uD
	c.state.uV = uV
	c.state.e = e
	return math.Max(c.MinOutput, math.Min(c.MaxOutput, uV))
}

func (c *TrackingController) GetState(now time.Time) *adv1.PIDState {
	return &adv1.PIDState{
		Time:            &timestamppb.Timestamp{Seconds: now.Unix(), Nanos: int32(now.Nanosecond())},
		Error:           float32(c.state.e),
		IntegralError:   float32(c.state.eI),
		IntegralState:   float32(c.state.uI),
		DerivativeState: float32(c.state.uD),
		ControlSignal:   float32(c.state.uV),
	}
}

func (c *TrackingController) DischargeIntegral(dt time.Duration) {
	c.state.eI = 0.0
	c.state.uI =
		math.Max(0, math.Min(1-dt.Seconds()*c.IntegralPartDecreaseFactor, 1.0)) * c.state.uI
}
