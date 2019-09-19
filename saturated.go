package pid

import (
	"math"
	"time"
)

// SaturatedPID implements a PIDT1-controller with feed forward term, a saturated control output
// and anti-windup functionality. The DT1-part behaves much like a D-part up to a tunable cut-off
// frequency. The anti-windup mechanism uses a tracking mode as defined in Chapter 6 of
// Åström and Murray, Feedback Systems: An Introduction to Scientists and Engineers, 2008
// (https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
// The complete controller structure is detailed in
// Steer-by-Wire Controller Development for Einride's T-Pod: Phase One, Michele Sigilló, 2018
// (https://drive.google.com/drive/u/0/folders/15lvDR3EwdNJOU402h-gINsvRl01OzY4T)

// Controller tuning parameters
// DerivativeGain - D part gain
// CutOffFrequency - D part low-pass filter time constant => cut-off frequency 1/CutOffFrequency
// ProportionalGain - P part gain
// IntegrationGain - I part gain
// WindUp - Anti-windup tracking gain
type SaturatedPID struct {
	DerivativeGain   float64
	CutOffFrequency  float64
	ProportionalGain float64
	IntegrationGain  float64
	WindUp           float64
	MaxOutput        float64
	MinOutput        float64
	state            saturatedPIDState
}

type saturatedPIDState struct {
	e  float64
	eI float64
	uI float64
	uD float64
	u  float64
}

func (c *SaturatedPID) Reset() {
	c.state = saturatedPIDState{}
}

func (c *SaturatedPID) Update(target float64, actual float64, ff float64, dt time.Duration) float64 {
	e := target - actual
	uP := e * c.ProportionalGain
	uI := c.state.eI*c.IntegrationGain*dt.Seconds() + c.state.uI
	uD := ((c.DerivativeGain/c.CutOffFrequency)*(e-c.state.e) + c.state.uD) / (dt.Seconds()/c.CutOffFrequency + 1)
	uV := uP + uI + uD + ff
	c.state.u = math.Max(c.MinOutput, math.Min(c.MaxOutput, uV))
	c.state.eI = e + c.WindUp*(c.state.u-uV)
	c.state.uI = uI
	c.state.uD = uD
	c.state.e = e
	return c.state.u
}
