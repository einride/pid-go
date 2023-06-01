package pid

import (
	"math"
	"testing"
	"time"

	"gotest.tools/v3/assert"
)

func TestTrackingController_PControllerUpdate(t *testing.T) {
	// Given a tracking P controller
	c := &TrackingController{
		Config: TrackingControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			ProportionalGain:              1,
			AntiWindUpGain:                0,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
		},
	}
	for _, tt := range []struct {
		input         TrackingControllerInput
		expectedState TrackingControllerState
	}{
		{
			input: TrackingControllerInput{
				ReferenceSignal:  1.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
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
			input: TrackingControllerInput{
				ReferenceSignal:  50.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
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
			input: TrackingControllerInput{
				ReferenceSignal:  -50.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
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
		c.Update(tt.input)
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.State)
		c.Reset()
	}
}

func TestTrackingController_NaN(t *testing.T) {
	// Given a saturated I controller with a low AntiWindUpGain
	c := &TrackingController{
		Config: TrackingControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			IntegralGain:                  10,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
			AntiWindUpGain:                0.01,
		},
	}
	for _, tt := range []struct {
		input         TrackingControllerInput
		expectedState TrackingControllerState
	}{
		{
			// Negative faulty measurement
			input: TrackingControllerInput{
				ReferenceSignal:   5.0,
				ActualSignal:      -math.MaxFloat64,
				FeedForwardSignal: 2.0,
				SamplingInterval:  dtTest,
			},
		},
		{
			// Positive faulty measurement
			input: TrackingControllerInput{
				ReferenceSignal:   5.0,
				ActualSignal:      math.MaxFloat64,
				FeedForwardSignal: 2.0,
				SamplingInterval:  dtTest,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		for i := 0; i < 220; i++ {
			c.Update(TrackingControllerInput{
				ReferenceSignal:   tt.input.ReferenceSignal,
				ActualSignal:      tt.input.ActualSignal,
				FeedForwardSignal: tt.input.FeedForwardSignal,
				SamplingInterval:  tt.input.SamplingInterval,
			})
		}
		// Then
		assert.Assert(t, !math.IsNaN(c.State.UnsaturatedControlSignal))
		assert.Assert(t, !math.IsNaN(c.State.ControlSignal))
		assert.Assert(t, !math.IsNaN(c.State.ControlErrorIntegral))
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
			LowPassTimeConstant:           1 * time.Second,
			ProportionalGain:              1,
			DerivativeGain:                10,
			IntegralGain:                  0.01,
			AntiWindUpGain:                0.5,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
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
