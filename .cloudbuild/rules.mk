gcp_project := einride
github_org := einride
repo_name := $(shell basename -s .git $(shell git config --get remote.origin.url))
git_root := $(shell git rev-parse --show-toplevel)
cloudbuild_root := $(shell realpath --relative-to $(git_root) $(dir $(lastword $(MAKEFILE_LIST))))

.PHONY: gcloud-builds-triggers-create
gcloud-builds-triggers-create:
	gcloud beta builds triggers create github \
		--project=$(gcp_project) \
		--repo-owner=$(github_org) \
		--repo-name=$(repo_name) \
		--pull-request-pattern='.*' \
		--description='$(repo_name)-review' \
		--build-config='$(cloudbuild_root)/review.yaml'
