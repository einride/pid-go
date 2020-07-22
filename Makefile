# all: run a complete build
all: \
	markdown-lint \
	go-lint \
	go-review \
	go-test \
	go-mod-tidy \
	git-verify-nodiff

include tools/git-verify-nodiff/rules.mk
include tools/golangci-lint/rules.mk
include tools/prettier/rules.mk
include tools/goreview/rules.mk

# go-mod-tidy: update Go module files
.PHONY: go-mod-tidy
go-mod-tidy:
	find . -name go.mod -execdir go mod tidy -v \;

# go-test: run Go test suite
.PHONY: go-test
go-test:
	go test -count 1 -cover -race ./...
