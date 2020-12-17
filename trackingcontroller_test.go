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
			LowPassTimeConstant:         1 * time.Second,
			ProportionalGain:            1,
			AntiWindUpGain:              0,
			IntegralPartDischargeFactor: 0.1,
			MinOutput:                   -10,
			MaxOutput:                   10,
		},
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  TrackingControllerState
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState: TrackingControllerState{
				ControlError:             1.0,
				ControlErrorIntegrand:    1.0,
				ControlErrorIntegral:     0.0,
				ControlErrorDerivative:   1.0 / (dtTest.Seconds() + c.Config.LowPassTimeConstant.Seconds()),
				ControlSignal:            1.0,
				UnsaturatedControlSignal: 1.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState: TrackingControllerState{
				ControlError:             50.0,
				ControlErrorIntegrand:    50.0,
				ControlErrorIntegral:     0.0,
				ControlErrorDerivative:   50.0 / (dtTest.Seconds() + c.Config.LowPassTimeConstant.Seconds()),
				ControlSignal:            10.0,
				UnsaturatedControlSignal: 50.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState: TrackingControllerState{
				ControlError:             -50.0,
				ControlErrorIntegrand:    -50.0,
				ControlErrorIntegral:     0.0,
				ControlErrorDerivative:   -50.0 / (dtTest.Seconds() + c.Config.LowPassTimeConstant.Seconds()),
				ControlSignal:            -10.0,
				UnsaturatedControlSignal: -50.0,
			},
		},
	} {
		tt := tt
		// When
		c.Update(tt.reference, tt.measuredOutput, 0.0, 0.0, dtTest)
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.State)
		c.Reset()
	}
}

func TestTrackingController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &TrackingController{}
	c.State = TrackingControllerState{
		ControlError:           5,
		ControlErrorIntegral:   5,
		ControlErrorDerivative: 5,
		ControlSignal:          5,
		ControlErrorIntegrand:  5,
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
			LowPassTimeConstant:         1 * time.Second,
			ProportionalGain:            1,
			DerivativeGain:              10,
			IntegralGain:                0.01,
			AntiWindUpGain:              0.5,
			IntegralPartDischargeFactor: 0.1,
			MinOutput:                   -10,
			MaxOutput:                   10,
		},
	}
	c.State = TrackingControllerState{
		ControlError:           5,
		ControlErrorIntegral:   100000,
		ControlErrorDerivative: 50,
		ControlSignal:          1,
		ControlErrorIntegrand:  10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	// Then
	expected := TrackingControllerState{
		ControlError:           5,
		ControlErrorIntegrand:  0.0,
		ControlErrorIntegral:   99900.0,
		ControlErrorDerivative: 50.0,
		ControlSignal:          1.0,
	}
	assert.Equal(t, expected, c.State)
}
