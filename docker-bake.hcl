group "default" {
  targets = ["build"]
}

target "build" {
  target = "build_stage"
  dockerfile = "Dockerfile"
  platforms = ["linux/arm64/v8"]
  tags = ["docker.io/hakuturu583/libtorch_vendor:build"]
  group = ["build"]
}
