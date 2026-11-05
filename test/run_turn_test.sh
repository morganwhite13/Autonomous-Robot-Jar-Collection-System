#!/bin/sh
# Compiles the controller against a minimal stand-in for the Webots controller
# API (test/stub) and runs TurnTest at several turn rates. No Webots needed.
# The realistic rate for a Pioneer 3-DX at MAX_SPEED with a 32 ms step is about 0.0094.
set -e
cd "$(dirname "$0")/.."
out=$(mktemp -d)
javac -nowarn -d "$out" $(find test/stub -name '*.java') ProjectController3.java test/TurnTest.java
for rate in 0.0047 0.0094 0.02; do
  java -cp "$out" TurnTest "$rate"
done
