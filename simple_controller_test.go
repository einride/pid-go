package pid

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
)

func TestSpeedControl_ControlLoop_OutputIncrease(t *testing.T) {
	// Given a pidControl with reference value and update interval, dt
	reference := float64(10)
	pidControl := SimpleController{
		ProportionalGain: 2.0,
		IntegrationGain:  1.0,
		DerivationGain:   1.0,
		OutputUpperLimit: 50,
	}
	// Check output value when output increase is needed
	assert.Equal(t, float64(50), pidControl.Update(reference, 0, time.Second/10))
	// Check proportional error
	assert.Equal(t, float64(10), pidControl.errorSize)
}

func TestSpeedControl_ControlLoop_OutputDecrease(t *testing.T) {
	// Given a pidControl with reference output and update interval, dt
	reference := float64(10)
	pidControl := SimpleController{
		ProportionalGain: 2.0,
		IntegrationGain:  1.0,
		DerivationGain:   1.0,
		OutputUpperLimit: 50,
	}
	// Check output value when output value decrease is needed
	assert.Equal(t, float64(0), pidControl.Update(reference, 10, time.Second/10))
	// Check proportional error
	assert.Equal(t, float64(0), pidControl.errorSize)
}

func TestSimpleController_Reset(t *testing.T) {
	// Given a SimpleController with stored values not equal to 0
	c := &SimpleController{
		ProportionalGain: 2.0,
		IntegrationGain:  1.0,
		DerivationGain:   1.0,
		OutputUpperLimit: 50,
		errorOverTime:    10,
		errorRate:        10,
		errorSize:        10,
	}
	// And a duplicate SimpleController with empty values
	expectedController := &SimpleController{
		ProportionalGain: 2.0,
		IntegrationGain:  1.0,
		DerivationGain:   1.0,
		OutputUpperLimit: 50,
	}

	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, expectedController, c)
}
