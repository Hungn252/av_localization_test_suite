#!/bin/bash
# build.sh — build and push the Docker image.
#
# Usage:
#   ./build.sh <registry/org> [tag]
#
# Examples:
#   ./build.sh ghcr.io/myorg
#   ./build.sh myuser          # Docker Hub
#   ./build.sh ghcr.io/myorg v1.0.0

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <registry/org> [tag]"
    echo ""
    echo "Examples:"
    echo "  $0 ghcr.io/myorg"
    echo "  $0 myuser           # Docker Hub"
    exit 1
fi

REGISTRY="$1"
TAG="${2:-latest}"
IMAGE="$REGISTRY/slam-suite:$TAG"
SUITE_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[build] Image:  $IMAGE"
echo "[build] Context: $SUITE_DIR"
echo ""

echo "[build] Building..."
docker build -t "$IMAGE" "$SUITE_DIR"

echo ""
echo "[build] Pushing $IMAGE ..."
docker push "$IMAGE"

echo ""
echo "[build] Done: $IMAGE"
echo ""
echo "Users can now run:"
echo "  docker pull $IMAGE"
echo "  docker run --rm --network host \\"
echo "    -v \$(pwd)/user_config.yaml:/root/suite/user_config.yaml:ro \\"
echo "    -v /path/to/results:/results \\"
echo "    $IMAGE"
