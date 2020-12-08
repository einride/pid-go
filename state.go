package pid

type State struct {
	Error           float64
	IntegralError   float64
	IntegralState   float64
	DerivativeState float64
	ControlSignal   float64
}
