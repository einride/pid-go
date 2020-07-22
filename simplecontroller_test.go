package pid

import (
	"testing"
	"time"

	"gotest.tools/v3/assert"
	is "gotest.tools/v3/assert/cmp"
)

func TestSpeedControl_ControlLoop_OutputIncrease(t *testing.T) {
	// Given a pidControl with reference value and update interval, dt
	reference := float64(10)
	pidControl := SimpleController{
		ProportionalGain: 2.0,
		IntegralGain:     1.0,
		DerivativeGain:   1.0,
		MaxOutput:        50,
	}
	// Check output value when output increase is needed
	assert.Check(t, is.Equal(float64(50), pidControl.Update(reference, 0, time.Second/10)))
	// Check proportional error
	assert.Check(t, is.Equal(float64(10), pidControl.state.errorSize))
}

func TestSpeedControl_ControlLoop_OutputDecrease(t *testing.T) {
	// Given a pidControl with reference output and update interval, dt
	reference := float64(10)
	pidControl := SimpleController{
		ProportionalGain: 2.0,
		IntegralGain:     1.0,
		DerivativeGain:   1.0,
		MaxOutput:        50,
	}
	// Check output value when output value decrease is needed
	assert.Check(t, is.Equal(float64(0), pidControl.Update(reference, 10, time.Second/10)))
	// Check proportional error
	assert.Check(t, is.Equal(float64(0), pidControl.state.errorSize))
}

func TestSimpleController_Reset(t *testing.T) {
	// Given a SimpleController with stored values not equal to 0
	c := &SimpleController{
		ProportionalGain: 2.0,
		IntegralGain:     1.0,
		DerivativeGain:   1.0,
		MaxOutput:        50,
		state: simpleState{
			errorOverTime: 10,
			errorRate:     10,
			errorSize:     10,
		},
	}
	// And a duplicate SimpleController with empty values
	expectedController := &SimpleController{
		ProportionalGain: 2.0,
		IntegralGain:     1.0,
		DerivativeGain:   1.0,
		MaxOutput:        50,
	}

	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, expectedController.state, c.state)
}
