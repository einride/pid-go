package pid

import (
	"testing"
	"time"

	"gotest.tools/v3/assert"
)

func TestTrackingController_PControllerUpdate(t *testing.T) {
	// Given a tracking P controller
	c := &TrackingController{
		Config: TrackingControllerConfig{
			LowPassTimeConstant:        1 * time.Second,
			ProportionalGain:           1,
			AntiWindUpGain:             0,
			IntegralPartDecreaseFactor: 0.1,
			MinOutput:                  -10,
			MaxOutput:                  10,
		},
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  TrackingControllerState
		expectedOutput float64
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState: TrackingControllerState{
				Error:           1.0,
				IntegralError:   1.0,
				IntegralState:   0.0,
				DerivativeState: 0.0,
				ControlSignal:   1.0,
			},
			expectedOutput: 1.0,
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState: TrackingControllerState{
				Error:           50.0,
				IntegralError:   50.0,
				IntegralState:   0.0,
				DerivativeState: 0.0,
				ControlSignal:   50.0,
			},
			expectedOutput: 10.0,
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState: TrackingControllerState{
				Error:           -50.0,
				IntegralError:   -50.0,
				IntegralState:   0.0,
				DerivativeState: 0.0,
				ControlSignal:   -50.0,
			},
			expectedOutput: -10.0,
		},
	} {
		tt := tt
		// When
		assert.Equal(t, tt.expectedOutput, c.Update(tt.reference, tt.measuredOutput, 0.0, 0.0, dtTest))
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.State)
	}
}

func TestTrackingController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &TrackingController{}
	c.State = TrackingControllerState{
		Error:           5,
		IntegralState:   5,
		DerivativeState: 5,
		ControlSignal:   5,
		IntegralError:   5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, TrackingControllerState{}, c.State)
}

func TestTrackingController_OffloadIntegralTerm(t *testing.T) {
	// Given a tracking PID controller
	c := &TrackingController{
		Config: TrackingControllerConfig{
			LowPassTimeConstant:        1 * time.Second,
			ProportionalGain:           1,
			DerivativeGain:             10,
			IntegralGain:               0.01,
			AntiWindUpGain:             0.5,
			IntegralPartDecreaseFactor: 0.1,
			MinOutput:                  -10,
			MaxOutput:                  10,
		},
	}
	c.State = TrackingControllerState{
		Error:           5,
		IntegralState:   1000,
		DerivativeState: 500,
		ControlSignal:   1,
		IntegralError:   10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	// Then
	expected := TrackingControllerState{
		Error:           5,
		IntegralError:   0.0,
		IntegralState:   999.0,
		DerivativeState: 500.0,
		ControlSignal:   1.0,
	}
	assert.Equal(t, expected, c.State)
}
