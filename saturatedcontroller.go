package pid

import (
	"math"
	"time"
)

// SaturatedController implements a PIDT1-controller with feed forward term, a saturated control output and anti-windup.
//
// The DT1-part behaves much like a D-part up to a tunable cut-off frequency.
//
// The anti-windup mechanism uses a tracking mode as defined in Chapter 6 of Åström and Murray, Feedback Systems:
// An Introduction to Scientists and Engineers, 2008
// (https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
//
// The complete controller structure is detailed in Steer-by-Wire Controller Development for Einride's T-Pod: Phase One,
// Michele Sigilló, 2018.
type SaturatedController struct {
	Config SaturatedControllerConfig
	State  SaturatedControllerState
}

type SaturatedControllerConfig struct {
	// ProportionalGain is the P part gain.
	ProportionalGain float64
	// IntegralGain is the I part gain.
	IntegralGain float64
	// DerivativeGain is the D part gain.
	DerivativeGain float64
	// AntiWindUpGain is the anti-windup tracking gain.
	AntiWindUpGain float64
	// Inverse of the time constant to discharge the integral part of the PID controller [1/s]
	IntegralPartDecreaseFactor float64
	// LowPassTimeConstant is the D part low-pass filter time constant => cut-off frequency 1/LowPassTimeConstant.
	LowPassTimeConstant time.Duration
	// MaxOutput is the max output from the PID.
	MaxOutput float64
	// MinOutput is the min output from the PID.
	MinOutput float64
}

type SaturatedControllerState struct {
	Error           float64
	IntegralError   float64
	IntegralState   float64
	DerivativeState float64
	ControlSignal   float64
}

func (c *SaturatedController) Reset() {
	c.State = SaturatedControllerState{}
}

func (c *SaturatedController) Update(target float64, actual float64, ff float64, dt time.Duration) float64 {
	e := target - actual
	uP := e * c.Config.ProportionalGain
	uI := c.State.IntegralError*c.Config.IntegralGain*dt.Seconds() + c.State.IntegralState
	uD := ((c.Config.DerivativeGain/c.Config.LowPassTimeConstant.Seconds())*(e-c.State.Error) +
		c.State.DerivativeState) /
		(dt.Seconds()/c.Config.LowPassTimeConstant.Seconds() + 1)
	uV := uP + uI + uD + ff
	c.State.ControlSignal = math.Max(c.Config.MinOutput, math.Min(c.Config.MaxOutput, uV))
	c.State.IntegralError = e + c.Config.AntiWindUpGain*(c.State.ControlSignal-uV)
	c.State.IntegralState = uI
	c.State.DerivativeState = uD
	c.State.Error = e
	return c.State.ControlSignal
}

func (c *SaturatedController) DischargeIntegral(dt time.Duration) {
	c.State.IntegralError = 0.0
	c.State.IntegralState = math.Max(
		0,
		math.Min(1-dt.Seconds()*c.Config.IntegralPartDecreaseFactor, 1.0),
	) * c.State.IntegralState
}
