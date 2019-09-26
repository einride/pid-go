package pid

import (
	"testing"
	"time"

	"github.com/stretchr/testify/require"
)

func TestTrackingController_PControllerUpdate(t *testing.T) {
	// Given a saturated P controller
	c := &TrackingController{
		LowPassTimeConstant: 1 * time.Second,
		ProportionalGain:    1,
		AntiWindUpGain:      0,
		MinOutput:           -10,
		MaxOutput:           10,
	}
	for _, tt := range []struct {
		measuredOutput float64
		reference      float64
		expectedState  trackingState
		expectedOutput float64
	}{
		{
			measuredOutput: 0.0,
			reference:      1.0,
			expectedState:  trackingState{e: 1.0, uI: 0.0, eI: 1.0},
			expectedOutput: 1.0,
		},
		{
			measuredOutput: 0.0,
			reference:      50.0,
			expectedState:  trackingState{e: 50.0, uI: 0.0, eI: 50.0},
			expectedOutput: 10.0,
		},
		{
			measuredOutput: 0.0,
			reference:      -50.0,
			expectedState:  trackingState{e: -50.0, uI: -0.0, eI: -50.0},
			expectedOutput: -10.0,
		},
	} {
		tt := tt
		// When
		require.Equal(t, tt.expectedOutput, c.Update(tt.reference, tt.measuredOutput, 0.0, 0.0, dtTest))
		// Then the controller state should be the expected
		require.Equal(t, tt.expectedState, c.state)
	}
}

func TestTrackingController_Reset(t *testing.T) {
	// Given a SaturatedPIDController with stored values not equal to 0
	c := &TrackingController{}
	c.state = trackingState{
		e:  5,
		uI: 5,
		uD: 5,
		eI: 5,
	}
	// When resetting stored values
	c.Reset()
	// Then
	require.Equal(t, trackingState{}, c.state)
}
