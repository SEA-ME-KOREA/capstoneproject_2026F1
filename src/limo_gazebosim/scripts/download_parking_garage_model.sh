#!/usr/bin/env bash

set -euo pipefail

MODEL_NAME="parking_garage"
MODEL_REPO_URL="https://github.com/osrf/gazebo_models.git"
TARGET_BASE_DIR="${1:-$HOME/.gazebo/models}"
TARGET_DIR="${TARGET_BASE_DIR}/${MODEL_NAME}"
TMP_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${TMP_DIR}"
}

trap cleanup EXIT

mkdir -p "${TARGET_BASE_DIR}"

if [ -d "${TARGET_DIR}" ]; then
  echo "Model already exists: ${TARGET_DIR}"
  exit 0
fi

git clone \
  --depth 1 \
  --filter=blob:none \
  --sparse \
  "${MODEL_REPO_URL}" \
  "${TMP_DIR}/gazebo_models"

git -C "${TMP_DIR}/gazebo_models" sparse-checkout set "${MODEL_NAME}"

if [ ! -d "${TMP_DIR}/gazebo_models/${MODEL_NAME}" ]; then
  echo "Failed to find model '${MODEL_NAME}' in ${MODEL_REPO_URL}" >&2
  exit 1
fi

cp -a "${TMP_DIR}/gazebo_models/${MODEL_NAME}" "${TARGET_DIR}"

echo "Installed ${MODEL_NAME} to ${TARGET_DIR}"
