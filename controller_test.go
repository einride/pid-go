package pid

import (
	"testing"
	"time"

	"gotest.tools/v3/assert"
)

func TestSpeedControl_ControlLoop_OutputIncrease(t *testing.T) {
	// Given a pidControl with reference value and update interval, dt
	pidControl := Controller{
		Config: ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
	}
	// Check output value when output increase is needed
	pidControl.Update(ControllerInput{
		ReferenceSignal:  10,
		ActualSignal:     0,
		SamplingInterval: 100 * time.Millisecond,
	})
	assert.Equal(t, float64(121), pidControl.State.ControlSignal)
	assert.Equal(t, float64(10), pidControl.State.ControlError)
}

func TestSpeedControl_ControlLoop_OutputDecrease(t *testing.T) {
	// Given a pidControl with reference output and update interval, dt
	pidControl := Controller{
		Config: ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
	}
	// Check output value when output value decrease is needed
	pidControl.Update(ControllerInput{
		ReferenceSignal:  10,
		ActualSignal:     10,
		SamplingInterval: 100 * time.Millisecond,
	})
	assert.Equal(t, float64(0), pidControl.State.ControlSignal)
	assert.Equal(t, float64(0), pidControl.State.ControlError)
}

func TestSimpleController_Reset(t *testing.T) {
	// Given a Controller with stored values not equal to 0
	c := &Controller{
		Config: ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
		State: ControllerState{
			ControlErrorIntegral:   10,
			ControlErrorDerivative: 10,
			ControlError:           10,
		},
	}
	// And a duplicate Controller with empty values
	expectedController := &Controller{
		Config: ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, expectedController.State, c.State)
}
