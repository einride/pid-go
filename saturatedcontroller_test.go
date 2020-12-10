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

func TestSaturatedController_PControllerUpdate(t *testing.T) {
	// Given a saturated P controller
	c := &SaturatedController{
		Config: SaturatedControllerConfig{
			LowPassTimeConstant:        1 * time.Second,
			ProportionalGain:           1,
			IntegralPartDecreaseFactor: 0.1,
			MinOutput:                  -10,
			MaxOutput:                  10,
		},
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  SaturatedControllerState
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState: SaturatedControllerState{
				ControlError:         1.0,
				ControlSignal:        c.Config.ProportionalGain * 1.0,
				ControlErrorIntegral: 1.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState: SaturatedControllerState{
				ControlError:         50.0,
				ControlSignal:        c.Config.MaxOutput,
				ControlErrorIntegral: 50.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState: SaturatedControllerState{
				ControlError:         -50.0,
				ControlSignal:        c.Config.MinOutput,
				ControlErrorIntegral: -50.0,
			},
		},
	} {
		tt := tt
		// When
		c.Update(tt.reference, tt.measuredOutput, 0.0, dtTest)
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.State)
	}
}

func TestSaturatedController_PIDUpdate(t *testing.T) {
	// Given a saturated PID controller
	c := &SaturatedController{
		Config: SaturatedControllerConfig{
			LowPassTimeConstant:        1 * time.Second,
			DerivativeGain:             0.01,
			ProportionalGain:           1,
			IntegralGain:               10,
			AntiWindUpGain:             10,
			IntegralPartDecreaseFactor: 0.1,
			MinOutput:                  -10,
			MaxOutput:                  10,
		},
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  SaturatedControllerState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			expectedState: SaturatedControllerState{
				ControlError:         0.0,
				ControlSignal:        5.0,
				ControlErrorIntegral: 0.0,
				DerivativeState:      0.0,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		for i := 0; i < 500; i++ {
			c.Update(tt.reference, c.State.ControlSignal, 0.0, dtTest)
		}
		// Then the controller I state only should give the expected output
		assert.Assert(t, math.Abs(tt.expectedState.ControlError-c.State.ControlError) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlSignal-c.State.ControlSignal) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlSignal-c.State.IntegralState) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlErrorIntegral-c.State.ControlErrorIntegral) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.DerivativeState-c.State.DerivativeState) < deltaTest)
	}
}

func TestSaturatedPID_FFUpdate(t *testing.T) {
	// Given a saturated I controller
	c := &SaturatedController{
		Config: SaturatedControllerConfig{
			LowPassTimeConstant:        1 * time.Second,
			IntegralGain:               10,
			IntegralPartDecreaseFactor: 0.1,
			MinOutput:                  -10,
			MaxOutput:                  10,
		},
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		feedForward    float64
		expectedState  SaturatedControllerState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    2.0,
			expectedState: SaturatedControllerState{
				ControlError:  0.0,
				ControlSignal: 5.0,
				IntegralState: 5.0 - 2.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    15.0,
			expectedState: SaturatedControllerState{
				ControlError:  0.0,
				ControlSignal: 5.0,
				IntegralState: 5.0 - 15.0,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		// Then the controller I state should compensate for difference between feed forward without
		// violating saturation constraints.
		for i := 0; i < 500; i++ {
			c.Update(tt.reference, c.State.ControlSignal, tt.feedForward, dtTest)
			assert.Assert(t, c.State.ControlSignal <= c.Config.MaxOutput)
		}
		assert.Assert(t, math.Abs(tt.expectedState.ControlError-c.State.ControlError) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.ControlSignal-c.State.ControlSignal) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.IntegralState-c.State.IntegralState) < deltaTest)
	}
}

func TestSaturatedController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &SaturatedController{}
	c.State = SaturatedControllerState{
		ControlError:         5,
		IntegralState:        5,
		DerivativeState:      5,
		ControlSignal:        5,
		ControlErrorIntegral: 5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, SaturatedControllerState{}, c.State)
}

func TestSaturatedController_OffloadIntegralTerm(t *testing.T) {
	// Given a saturated PID controller
	c := &SaturatedController{
		Config: SaturatedControllerConfig{
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
	c.State = SaturatedControllerState{
		ControlError:         5,
		IntegralState:        1000,
		DerivativeState:      500,
		ControlSignal:        1,
		ControlErrorIntegral: 10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	expected := SaturatedControllerState{
		ControlError:         5,
		ControlErrorIntegral: 0.0,
		IntegralState:        999.0,
		DerivativeState:      500.0,
		ControlSignal:        1.0,
	}
	// Then
	assert.Equal(t, c.State, expected)
}
