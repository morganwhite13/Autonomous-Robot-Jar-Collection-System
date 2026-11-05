#!/usr/bin/env bash
# Mutation testing (PIT) for v2's logic classes: makes small changes to the code (flips a
# comparison, removes a call, returns a constant...) and checks that some test fails for each.
# A change no test notices ("survived") points at behaviour the tests don't pin down.
#
# Lines that only call Log are skipped (--avoidCallsTo Log): the log text isn't behaviour.
# Fails if fewer than MIN_SCORE percent of the changes are caught. The HTML report is written
# to build/pit/index.html. Needs a JDK and curl; runs tools/run-tests.sh first.
set -euo pipefail
cd "$(dirname "$0")/.."

MIN_SCORE=${MIN_SCORE:-90}
LIB=build/lib
MIRRORS=(https://repo1.maven.org/maven2 https://maven-central.storage-download.googleapis.com/maven2)

# path in the Maven repository, then the jar's SHA-256
JARS=(
  "org/pitest/pitest/1.15.8/pitest-1.15.8.jar c4f8024a2faf52f3cf5d205b47b1e18fe3cdc30503aaa9ecf75fe67cf3b0ff76"
  "org/pitest/pitest-entry/1.15.8/pitest-entry-1.15.8.jar 6cd71474c6bef7e05d7d53086253ded57c9f9a2e636ee99c39cfe96ec5131752"
  "org/pitest/pitest-command-line/1.15.8/pitest-command-line-1.15.8.jar f1126c5b834b3c8c09f5d6c44a4e38929001283886f2328beaf6e12132d0af2e"
  "org/pitest/pitest-junit5-plugin/1.2.1/pitest-junit5-plugin-1.2.1.jar 9d944887a583ebd4adc33d2d26b8b56639fd53fee93b4d068a90596705c072b5"
  "org/apache/commons/commons-text/1.10.0/commons-text-1.10.0.jar 770cd903fa7b604d1f7ef7ba17f84108667294b2b478be8ed1af3bffb4ae0018"
  "org/apache/commons/commons-lang3/3.13.0/commons-lang3-3.13.0.jar 82f528cf718c7a3c2f30fc5bc784e3c6a0a10b17605dadb9e16c82ede11e6064"
)

tools/run-tests.sh

mkdir -p "$LIB"
CP=""
for entry in "${JARS[@]}"; do
  read -r path sha <<< "$entry"
  jar="$LIB/$(basename "$path")"
  if [ ! -f "$jar" ]; then
    echo "==> Downloading $(basename "$path")"
    for mirror in "${MIRRORS[@]}"; do
      curl -fsSL --retry 4 --retry-delay 10 --retry-all-errors -o "$jar" "$mirror/$path" && break
      rm -f "$jar"
    done
  fi
  if ! echo "$sha  $jar" | sha256sum --check --status; then
    rm -f "$jar"
    echo "Checksum mismatch for $jar" >&2
    exit 1
  fi
  CP="$CP$jar:"
done
JUNIT_JAR="$LIB/junit-platform-console-standalone-1.10.2.jar"
LANG_JARS="$LIB/commons-text-1.10.0.jar,$LIB/commons-lang3-3.13.0.jar"

TARGETS='Angles,Vec2,RgbImage,Vision,Vision$*,Navigator,Navigator$*,JarHandler,JarHandler$*'
TARGETS="$TARGETS,Mission,MissionStats,MissionStats\$*,Routes,Routes\$*,RoutePlanner,ArenaMap"

echo "==> Mutation testing v2 (PIT); full output in build/pit.log"
rm -rf build/pit
if ! java -cp "$CP$JUNIT_JAR:build/test" org.pitest.mutationtest.commandline.MutationCoverageReport \
  --reportDir build/pit --timestampedReports=false --outputFormats HTML,CSV \
  --sourceDirs controllers/JarCollectorV2 --targetClasses "$TARGETS" --targetTests '*Test' \
  --classPath "$JUNIT_JAR,$LANG_JARS,build/test" --avoidCallsTo Log \
  --threads 4 --mutationThreshold "$MIN_SCORE" > build/pit.log 2>&1; then
  grep -E '^>> (Generated [0-9]+ mutations|Line)' build/pit.log || true
  grep -E 'threshold|Exception|did not pass' build/pit.log | head -20 >&2
  echo "Mutation testing failed (minimum score: $MIN_SCORE%)" >&2
  exit 1
fi
grep -E '^>> (Generated [0-9]+ mutations|Line)' build/pit.log

echo "==> Changes no test caught (all reviewed in docs/BEFORE_AND_AFTER.md):"
awk -F, '$6 == "SURVIVED" { sub(/.*\./, "", $3); print "    " $1 ":" $5 "  " $4 "  " $3 }' \
  build/pit/mutations.csv | sort -t: -k1,1 -k2,2n
