#!/bin/sh
# Check out the requested branch and run a module against the mounted corpus.
#
# Arguments: <module> [module arguments...]
set -eu

: "${PARTNET_MINING_REPOSITORY_URL:?repository url not set}"
: "${PARTNET_MINING_BRANCH:?branch not set}"
: "${PARTNET_MINING_DATASET_DIRECTORY:?dataset directory not set}"

if [ "$#" -eq 0 ]; then
    echo "usage: <module> [module arguments...]" >&2
    exit 2
fi

REPOSITORY_DIRECTORY=/work/repo
git clone --depth 1 --single-branch \
    --branch "${PARTNET_MINING_BRANCH}" \
    "${PARTNET_MINING_REPOSITORY_URL}" \
    "${REPOSITORY_DIRECTORY}"

# semantic_digital_twin imports giskardpy, which is a workspace package rather
# than a published one, so it comes from the same checkout.
export PYTHONPATH="${REPOSITORY_DIRECTORY}/krrood/src:${REPOSITORY_DIRECTORY}/semantic_digital_twin/src:${REPOSITORY_DIRECTORY}/giskardpy/src"
export PARTNET_MOBILITY_DATASET_DIRECTORY="${PARTNET_MINING_DATASET_DIRECTORY}"

cd "${REPOSITORY_DIRECTORY}"
exec python -m "$@"
