# Introduction PID-go

This repo contains different types of PID controllers:

- `simplecontroller` a PID controller
- `saturatedcontroller` a PIDT1-controller with feed forward term, a saturated control output and anti-windup
- `trackingcontroller` a PIDT1 controller with feed forward term, anti-windup and bumpless
  transfer using tracking mode control

All controllers are structured in the same way as an interface which can be seen in `controller.go`.

# Getting started

# FAQ

# References

`saturatedcontroller` and `trackingcontroller`:
https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf
