package main

import (
	"context"

	"go.einride.tech/sage/sg"
	"go.einride.tech/sage/tools/sgconvco"
	"go.einride.tech/sage/tools/sggit"
	"go.einride.tech/sage/tools/sggo"
	"go.einride.tech/sage/tools/sggolangcilint"
	"go.einride.tech/sage/tools/sggolicenses"
	"go.einride.tech/sage/tools/sggoreview"
	"go.einride.tech/sage/tools/sgmarkdownfmt"
	"go.einride.tech/sage/tools/sgyamlfmt"
)

func main() {
	sg.GenerateMakefiles(
		sg.Makefile{
			Path:          sg.FromGitRoot("Makefile"),
			DefaultTarget: Default,
		},
	)
}

func Default(ctx context.Context) error {
	sg.Deps(ctx, ConvcoCheck, FormatMarkdown, FormatYaml)
	sg.Deps(ctx, GoLint, GoReview)
	sg.Deps(ctx, GoTest)
	sg.Deps(ctx, GoModTidy)
	sg.Deps(ctx, GoLicenses, GitVerifyNoDiff)
	return nil
}

func GoModTidy(ctx context.Context) error {
	sg.Logger(ctx).Println("tidying Go module files...")
	return sg.Command(ctx, "go", "mod", "tidy", "-v").Run()
}

func GoTest(ctx context.Context) error {
	sg.Logger(ctx).Println("running Go tests...")
	return sggo.TestCommand(ctx).Run()
}

func GoReview(ctx context.Context) error {
	sg.Logger(ctx).Println("reviewing Go files...")
	return sggoreview.Run(ctx)
}

func GoLint(ctx context.Context) error {
	sg.Logger(ctx).Println("linting Go files...")
	return sggolangcilint.Run(ctx)
}

func GoLicenses(ctx context.Context) error {
	sg.Logger(ctx).Println("checking Go licenses...")
	return sggolicenses.Check(ctx)
}

func FormatMarkdown(ctx context.Context) error {
	sg.Logger(ctx).Println("formatting Markdown files...")
	return sgmarkdownfmt.Command(ctx, "-w", ".").Run()
}

func FormatYaml(ctx context.Context) error {
	sg.Logger(ctx).Println("formatting Yaml files...")
	return sgyamlfmt.Run(ctx)
}

func ConvcoCheck(ctx context.Context) error {
	sg.Logger(ctx).Println("checking git commits...")
	return sgconvco.Command(ctx, "check", "origin/master..HEAD").Run()
}

func GitVerifyNoDiff(ctx context.Context) error {
	sg.Logger(ctx).Println("verifying that git has no diff...")
	return sggit.VerifyNoDiff(ctx)
}
