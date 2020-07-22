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
		LowPassTimeConstant:        1 * time.Second,
		ProportionalGain:           1,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  saturatedState
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState:  saturatedState{e: 1.0, u: c.ProportionalGain * 1.0, eI: 1.0},
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState:  saturatedState{e: 50.0, u: c.MaxOutput, eI: 50.0},
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState:  saturatedState{e: -50.0, u: c.MinOutput, eI: -50.0},
		},
	} {
		tt := tt
		// When
		c.Update(tt.reference, tt.measuredOutput, 0.0, dtTest)
		// Then the controller state should be the expected
		assert.Equal(t, tt.expectedState, c.state)
	}
}

func TestSaturatedController_PIDUpdate(t *testing.T) {
	// Given a saturated PID controller
	c := &SaturatedController{
		LowPassTimeConstant:        1 * time.Second,
		DerivativeGain:             0.01,
		ProportionalGain:           1,
		IntegralGain:               10,
		AntiWindUpGain:             10,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  saturatedState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			expectedState: saturatedState{
				e:  0.0,
				u:  5.0,
				eI: 0.0,
				uD: 0.0,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		for i := 0; i < 500; i++ {
			c.Update(tt.reference, c.state.u, 0.0, dtTest)
		}
		// Then the controller I state only should give the expected output
		assert.Assert(t, math.Abs(tt.expectedState.e-c.state.e) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.u-c.state.u) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.u-c.state.uI) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.eI-c.state.eI) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.uD-c.state.uD) < deltaTest)
	}
}

func TestSaturatedPID_FFUpdate(t *testing.T) {
	// Given a saturated I controller
	c := &SaturatedController{
		LowPassTimeConstant:        1 * time.Second,
		IntegralGain:               10,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		feedForward    float64
		expectedState  saturatedState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    2.0,
			expectedState: saturatedState{
				e:  0.0,
				u:  5.0,
				uI: 5.0 - 2.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    15.0,
			expectedState: saturatedState{
				e:  0.0,
				u:  5.0,
				uI: 5.0 - 15.0,
			},
		},
	} {
		tt := tt
		// When enough iterations have passed
		c.Reset()
		// Then the controller I state should compensate for difference between feed forward without
		// violating saturation constraints.
		for i := 0; i < 500; i++ {
			c.Update(tt.reference, c.state.u, tt.feedForward, dtTest)
			assert.Assert(t, c.state.u <= c.MaxOutput)
		}
		assert.Assert(t, math.Abs(tt.expectedState.e-c.state.e) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.u-c.state.u) < deltaTest)
		assert.Assert(t, math.Abs(tt.expectedState.uI-c.state.uI) < deltaTest)
	}
}

func TestSaturatedController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &SaturatedController{}
	c.state = saturatedState{
		e:  5,
		uI: 5,
		uD: 5,
		u:  5,
		eI: 5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	assert.Equal(t, saturatedState{}, c.state)
}

func TestSaturatedController_OffloadIntegralTerm(t *testing.T) {
	// Given a saturated PID controller
	c := &SaturatedController{
		LowPassTimeConstant:        1 * time.Second,
		ProportionalGain:           1,
		DerivativeGain:             10,
		IntegralGain:               0.01,
		AntiWindUpGain:             0.5,
		IntegralPartDecreaseFactor: 0.1,
		MinOutput:                  -10,
		MaxOutput:                  10,
	}
	c.state = saturatedState{
		e:  5,
		uI: 1000,
		uD: 500,
		u:  1,
		eI: 10,
	}
	// When offloading the integral term
	c.DischargeIntegral(dtTest)
	// Then
	assert.Equal(t, c.state, saturatedState{e: 5, eI: 0.0, uI: 999.0, uD: 500.0, u: 1.0})
}
