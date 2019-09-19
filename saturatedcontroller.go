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
	// ProportionalGain is the P part gain.
	ProportionalGain float64
	// IntegralGain is the I part gain.
	IntegralGain float64
	// DerivativeGain is the D part gain.
	DerivativeGain float64
	// AntiWindUpGain is the anti-windup tracking gain.
	AntiWindUpGain float64
	// LowPassTimeConstant is the D part low-pass filter time constant => cut-off frequency 1/LowPassTimeConstant.
	LowPassTimeConstant time.Duration
	// MaxOutput is the max output from the PID.
	MaxOutput float64
	// MinOutput is the min output from the PID.
	MinOutput float64

	state saturatedState
}

type saturatedState struct {
	e  float64
	eI float64
	uI float64
	uD float64
	u  float64
}

func (c *SaturatedController) Reset() {
	c.state = saturatedState{}
}

func (c *SaturatedController) Update(target float64, actual float64, ff float64, dt time.Duration) float64 {
	e := target - actual
	uP := e * c.ProportionalGain
	uI := c.state.eI*c.IntegralGain*dt.Seconds() + c.state.uI
	uD := ((c.DerivativeGain/c.LowPassTimeConstant.Seconds())*(e-c.state.e) + c.state.uD) /
		(dt.Seconds()/c.LowPassTimeConstant.Seconds() + 1)
	uV := uP + uI + uD + ff
	c.state.u = math.Max(c.MinOutput, math.Min(c.MaxOutput, uV))
	c.state.eI = e + c.AntiWindUpGain*(c.state.u-uV)
	c.state.uI = uI
	c.state.uD = uD
	c.state.e = e
	return c.state.u
}
