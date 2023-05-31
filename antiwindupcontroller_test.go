package pid

import (
	"math"
	"testing"
	"time"

	"gotest.tools/v3/assert"
)

const (
	dtTest    = 10 * time.Millisecond
	deltaTest = 1e-3
)

func TestAntiWindupController_PControllerUpdate(t *testing.T) {
	// Given a saturated P controller
	c := &AntiWindupController{
		Config: AntiWindupControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			ProportionalGain:              1,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
		},
	}
	for _, tt := range []struct {
		input                 AntiWindupControllerInput
		expectedControlSignal float64
	}{
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:  1.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
			expectedControlSignal: c.Config.ProportionalGain * 1.0,
		},
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:  50.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
			expectedControlSignal: c.Config.MaxOutput,
		},
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:  -50.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
			expectedControlSignal: c.Config.MinOutput,
		},
	} {
		tt := tt
		// When
		c.Update(tt.input)
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedControlSignal, c.State.ControlSignal)
	}
}

func TestAntiWindupController_PIDUpdate(t *testing.T) {
	// Given a saturated PID controller
	c := &AntiWindupController{
		Config: AntiWindupControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			DerivativeGain:                0.01,
			ProportionalGain:              1,
			IntegralGain:                  10,
			AntiWindUpGain:                10,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
		},
	}
	for _, tt := range []struct {
		input         AntiWindupControllerInput
		expectedState AntiWindupControllerState
	}{
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:  5.0,
				ActualSignal:     0.0,
				SamplingInterval: dtTest,
			},
			expectedState: AntiWindupControllerState{
				ControlError:           0.0,
				ControlSignal:          5.0,
				ControlErrorIntegrand:  0.0,
				ControlErrorDerivative: 0.0,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		for i := 0; i < 500; i++ {
			c.Update(AntiWindupControllerInput{
				ReferenceSignal:   tt.input.ReferenceSignal,
				ActualSignal:      c.State.ControlSignal,
				FeedForwardSignal: 0,
				SamplingInterval:  tt.input.SamplingInterval,
			})
		}
		// Then the controller I state only should give the expected output
		assert.Assert(t, math.Abs(tt.expectedState.ControlError-c.State.ControlError) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlSignal-c.State.ControlSignal) < deltaTest)
		deltaI := math.Abs(tt.expectedState.ControlSignal - c.State.ControlErrorIntegral*c.Config.IntegralGain)
		assert.Assert(t, deltaI < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlErrorIntegrand-c.State.ControlErrorIntegrand) < deltaTest)
		deltaD := math.Abs(tt.expectedState.ControlErrorDerivative - c.State.ControlErrorDerivative*c.Config.DerivativeGain)
		assert.Assert(t, deltaD < deltaTest)
	}
}

func TestAntiWindupPID_FFUpdate(t *testing.T) {
	// Given a saturated I controller
	c := &AntiWindupController{
		Config: AntiWindupControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			IntegralGain:                  10,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
		},
	}
	for _, tt := range []struct {
		input         AntiWindupControllerInput
		expectedState AntiWindupControllerState
	}{
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:   5.0,
				ActualSignal:      0.0,
				FeedForwardSignal: 2.0,
				SamplingInterval:  dtTest,
			},
			expectedState: AntiWindupControllerState{
				ControlError:         0.0,
				ControlSignal:        5.0,
				ControlErrorIntegral: 0.5 - 0.2,
			},
		},
		{
			input: AntiWindupControllerInput{
				ReferenceSignal:   5.0,
				ActualSignal:      0.0,
				FeedForwardSignal: 15.0,
				SamplingInterval:  dtTest,
			},
			expectedState: AntiWindupControllerState{
				ControlError:         0.0,
				ControlSignal:        5.0,
				ControlErrorIntegral: 0.5 - 1.5,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		// Then the controller I state should compensate for difference between feed forward without
		// violating saturation constraints.
		for i := 0; i < 500; i++ {
			c.Update(AntiWindupControllerInput{
				ReferenceSignal:   tt.input.ReferenceSignal,
				ActualSignal:      c.State.ControlSignal,
				FeedForwardSignal: tt.input.FeedForwardSignal,
				SamplingInterval:  tt.input.SamplingInterval,
			})
			assert.Assert(t, c.State.ControlSignal <= c.Config.MaxOutput)
		}
		assert.Assert(t, math.Abs(tt.expectedState.ControlError-c.State.ControlError) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlSignal-c.State.ControlSignal) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlErrorIntegral-c.State.ControlErrorIntegral) < deltaTest)
	}
}

func TestAntiWindupPID_NaN(t *testing.T) {
	// Given a saturated I controller with a low AntiWindUpGain
	c := &AntiWindupController{
		Config: AntiWindupControllerConfig{
			LowPassTimeConstant:           1 * time.Second,
			IntegralGain:                  10,
			IntegralDischargeTimeConstant: 10,
			MinOutput:                     -10,
			MaxOutput:                     10,
			AntiWindUpGain:                0.01,
		},
	}
	for _, tt := range []struct {
		input         AntiWindupControllerInput
		expectedState AntiWindupControllerState
	}{
		{
			// Negative faulty measurement
			input: AntiWindupControllerInput{
				ReferenceSignal:   5.0,
				ActualSignal:      -math.MaxFloat64,
				FeedForwardSignal: 2.0,
				SamplingInterval:  dtTest,
			},
		},
		{
			// Positive faulty measurement
			input: AntiWindupControllerInput{
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
			c.Update(AntiWindupControllerInput{
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

func TestAntiWindupController_Reset(t *testing.T) {
	// Given a AntiWindupPIDController with stored values not equal to 0
	c := &AntiWindupController{}
	c.State = AntiWindupControllerState{
		ControlError:           5,
		ControlErrorIntegral:   5,
		ControlErrorDerivative: 5,
		ControlSignal:          5,
		ControlErrorIntegrand:  5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, AntiWindupControllerState{}, c.State)
}

func TestAntiWindupController_OffloadIntegralTerm(t *testing.T) {
	// Given a saturated PID controller
	c := &AntiWindupController{
		Config: AntiWindupControllerConfig{
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
	c.State = AntiWindupControllerState{
		ControlError:           5,
		ControlErrorIntegral:   1000,
		ControlErrorDerivative: 500,
		ControlSignal:          1,
		ControlErrorIntegrand:  10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	expected := AntiWindupControllerState{
		ControlError:           5,
		ControlErrorIntegrand:  0.0,
		ControlErrorIntegral:   999.0,
		ControlErrorDerivative: 500.0,
		ControlSignal:          1.0,
	}
	// Then
	assert.Equal(t, c.State, expected)
}
