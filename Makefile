# all: run a complete build
all: \
	markdown-lint \
	go-lint \
	go-review \
	go-test \
	go-mod-tidy \
	git-verify-nodiff \
	git-verify-submodules

# clean: remove all generated build files
.PHONY: clean
clean:
	rm -rf build

.PHONY: build
build:
	@git submodule update --init --recursive $@

include build/rules.mk
build/rules.mk: build
	@# included in submodule: build

# markdown-lint: lint Markdown files
.PHONY: markdown-lint
markdown-lint: $(MARKDOWNLINT)
	$(MARKDOWNLINT) --ignore build .

# go-mod-tidy: update Go module files
.PHONY: go-mod-tidy
go-mod-tidy:
	go mod tidy -v

# go-lint: lint Go files
.PHONY: go-lint
go-lint: $(GOLANGCI_LINT)
	$(GOLANGCI_LINT) run --enable-all

# go-test: run Go test suite
.PHONY: go-test
go-test:
	go test -count 1 -cover -race ./...

# go-review: review Go files
.PHONY: go-review
go-review: $(GOREVIEW)
	$(GOREVIEW) -c 1 ./...
