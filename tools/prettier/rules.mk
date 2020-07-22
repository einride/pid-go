prettier_version := 1.19.1
prettier_cwd := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))/v$(prettier_version)
prettier := $(prettier_cwd)/node_modules/.bin/prettier

$(prettier):
	$(info installing prettier...)
	@npm install --no-save --no-audit --prefix $(prettier_cwd) prettier@$(prettier_version)
	@chmod +x $@
	@touch $@

# markdown-lint: lint Markdown files
.PHONY: markdown-lint
markdown-lint: $(prettier)
	$(prettier) --check **/*.md --parser markdown
