# PID Go

[![PkgGoDev](https://pkg.go.dev/badge/go.einride.tech/pid)](https://pkg.go.dev/go.einride.tech/pid)
[![GoReportCard](https://goreportcard.com/badge/go.einride.tech/pid)](https://goreportcard.com/report/go.einride.tech/pid)
[![Codecov](https://codecov.io/gh/einride/pid-go/branch/master/graph/badge.svg)](https://codecov.io/gh/einride/pid-go)

<p align="center">
  <img src="./doc/pid-go.svg" alt="logo"/>
</p>

PID controllers for Go.

## Examples

### `pid.Controller`

A basic PID controller.

```go
import (
	"fmt"
	"time"

	"go.einride.tech/pid"
)

func ExampleController() {
	// Create a PID controller.
	c := pid.Controller{
		Config: pid.ControllerConfig{
			ProportionalGain: 2.0,
			IntegralGain:     1.0,
			DerivativeGain:   1.0,
		},
	}
	// Update the PID controller.
	c.Update(pid.ControllerInput{
		ReferenceSignal:  10,
		ActualSignal:     0,
		SamplingInterval: 100 * time.Millisecond,
	})
	fmt.Printf("%+v\n", c.State)
	// Reset the PID controller.
	c.Reset()
	fmt.Printf("%+v\n", c.State)
	// Output:
	// {ControlError:10 ControlErrorIntegral:1 ControlErrorDerivative:100 ControlSignal:121}
	// {ControlError:0 ControlErrorIntegral:0 ControlErrorDerivative:0 ControlSignal:0}
}
```

\_[Reference ≫](https://en.wikipedia.org/wiki/PID_controller)\_

### `pid.AntiWindupController`

A PID-controller with low-pass filtering of the derivative term, feed forward
term, a saturated control output and anti-windup.

*[Reference ≫](http://www.cds.caltech.edu/~murray/amwiki)*

### `pid.TrackingController`

a PID-controller with low-pass filtering of the derivative term, feed forward
term, anti-windup and bumpless transfer using tracking mode control.

*[Reference ≫](http://www.cds.caltech.edu/~murray/amwiki)*
