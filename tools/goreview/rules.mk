goreview_cwd := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
goreview := $(goreview_cwd)/bin/goreview

$(goreview): $(goreview_cwd)/go.mod
	@echo building goreview...
	@cd $(goreview_cwd) && go build -o $@ github.com/einride/goreview/cmd/goreview
	@cd $(goreview_cwd) && go mod tidy

# go-review: review Go code for Einride-specific conventions
.PHONY: go-review
go-review: $(goreview)
	@echo reviewing Go code for Einride-specific conventions with goreview...
	@$(goreview) -c 1 ./...
