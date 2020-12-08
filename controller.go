// Package pid provides facilities for steering using a PID control loop mechanism.
package pid

import (
	"time"
)

// Controller represents a PID controller with feed forward term.
type Controller interface {
	Update(targetValue float64, actualValue float64, ff float64, dt time.Duration) float64
	GetState() *State
	Reset()
	DischargeIntegral(dt time.Duration)
}

// BumplessController represents a PID controller with feed forward term and bumpless transfer between
// PID control and other controllers.
type BumplessController interface {
	Update(targetValue float64, actualValue float64, ff float64, actualInput float64, dt time.Duration) float64
	GetState() *State
	Reset()
	DischargeIntegral(dt time.Duration)
}
