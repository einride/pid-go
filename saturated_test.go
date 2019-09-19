package pid

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

const (
	dtTest    = 10 * time.Millisecond
	deltaTest = 1e-3
)

func TestSaturatedController_PControllerUpdate(t *testing.T) {
	// Given a saturated P controller
	c := &SaturatedPID{
		CutOffFrequency:  1,
		ProportionalGain: 1,
		MinOutput:        -10,
		MaxOutput:        10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  saturatedPIDState
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState:  saturatedPIDState{e: 1.0, u: c.ProportionalGain * 1.0, eI: 1.0},
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState:  saturatedPIDState{e: 50.0, u: c.MaxOutput, eI: 50.0},
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState:  saturatedPIDState{e: -50.0, u: c.MinOutput, eI: -50.0},
		},
	} {
		tt := tt
		// When
		c.Update(tt.reference, tt.measuredOutput, 0.0, dtTest)
		// Then the controller state should be the expected
		require.Equal(t, tt.expectedState, c.state)
	}
}

func TestSaturatedController_PIDUpdate(t *testing.T) {
	// Given a saturated PID controller
	c := &SaturatedPID{
		CutOffFrequency:  1,
		DerivativeGain:   0.01,
		ProportionalGain: 1,
		IntegrationGain:  10,
		WindUp:           10,
		MinOutput:        -10,
		MaxOutput:        10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  saturatedPIDState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			expectedState: saturatedPIDState{
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
		require.InDelta(t, tt.expectedState.e, c.state.e, deltaTest)
		require.InDelta(t, tt.expectedState.u, c.state.u, deltaTest)
		require.InDelta(t, tt.expectedState.u, c.state.uI, deltaTest)
		require.InDelta(t, tt.expectedState.eI, c.state.eI, deltaTest)
		require.InDelta(t, tt.expectedState.uD, c.state.uD, deltaTest)
	}
}

func TestSaturatedPID_FFUpdate(t *testing.T) {
	// Given a saturated I controller
	c := &SaturatedPID{
		CutOffFrequency: 1,
		IntegrationGain: 10,
		MinOutput:       -10,
		MaxOutput:       10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		feedForward    float64
		expectedState  saturatedPIDState
	}{
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    2.0,
			expectedState: saturatedPIDState{
				e:  0.0,
				u:  5.0,
				uI: 5.0 - 2.0,
			},
		},
		{
			measuredOutput: 0.0,
			reference:      5.0,
			feedForward:    15.0,
			expectedState: saturatedPIDState{
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
			require.True(t, c.state.u <= c.MaxOutput)
		}
		require.InDelta(t, tt.expectedState.e, c.state.e, deltaTest)
		require.InDelta(t, tt.expectedState.u, c.state.u, deltaTest)
		require.InDelta(t, tt.expectedState.uI, c.state.uI, deltaTest)
	}
}

func TestSaturatedController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &SaturatedPID{}
	c.state = saturatedPIDState{
		e:  5,
		uI: 5,
		uD: 5,
		u:  5,
		eI: 5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	require.Equal(t, saturatedPIDState{}, c.state)
}
