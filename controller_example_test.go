package pid_test

import (
	"fmt"
	"time"

	"github.com/einride/pid-go"
)

func ExampleController() {
	// Create a PID controller.
	c := pid.Controller{
		Config: pid.ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
			MaxOutput:        50,
		},
	}
	// Update the PID controller.
	c.Update(10, 0, 100*time.Millisecond)
	fmt.Printf("%+v\n", c.State)
	// Reset the PID controller.
	c.Reset()
	fmt.Printf("%+v\n", c.State)
	// Output:
	// {ControlError:10 ControlErrorIntegral:1 ControlErrorDerivative:100 ControlSignal:50}
	// {ControlError:0 ControlErrorIntegral:0 ControlErrorDerivative:0 ControlSignal:0}
}
