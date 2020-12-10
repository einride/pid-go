package pid

import (
	"testing"
	"time"

	"gotest.tools/v3/assert"
)

func TestTrackingController_PControllerUpdate(t *testing.T) {
	// Given a tracking P controller
	c := &TrackingController{
		LowPassTimeConstant:        1 * time.Second,
		ProportionalGain:           1,
		AntiWindUpGain:             0,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  trackingControllerState
		expectedOutput float64
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState:  trackingControllerState{e: 1.0, eI: 1.0, uI: 0.0, uD: 0.0, uV: 1.0},
			expectedOutput: 1.0,
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState:  trackingControllerState{e: 50.0, eI: 50.0, uI: 0.0, uD: 0.0, uV: 50.0},
			expectedOutput: 10.0,
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState:  trackingControllerState{e: -50.0, eI: -50.0, uI: 0.0, uD: 0.0, uV: -50.0},
			expectedOutput: -10.0,
		},
	} {
		tt := tt
		// When
		assert.Equal(t, tt.expectedOutput, c.Update(tt.reference, tt.measuredOutput, 0.0, 0.0, dtTest))
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.state)
	}
}

func TestTrackingController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &TrackingController{}
	c.state = trackingControllerState{
		e:  5,
		uI: 5,
		uD: 5,
		uV: 5,
		eI: 5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, trackingControllerState{}, c.state)
}

func TestTrackingController_OffloadIntegralTerm(t *testing.T) {
	// Given a tracking PID controller
	c := &TrackingController{
		LowPassTimeConstant:        1 * time.Second,
		ProportionalGain:           1,
		DerivativeGain:             10,
		IntegralGain:               0.01,
		AntiWindUpGain:             0.5,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	c.state = trackingControllerState{
		e:  5,
		uI: 1000,
		uD: 500,
		uV: 1,
		eI: 10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	// Then
	assert.Equal(t, c.state, trackingControllerState{e: 5, eI: 0.0, uI: 999.0, uD: 500.0, uV: 1.0})
}
