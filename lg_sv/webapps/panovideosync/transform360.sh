#!/usr/bin/env bash

set -e

echo "Usage: `basename $0` <input_vid> <output_vid>"

INPUT_VID=$1
OUTPUT_VID=$2

test -e "${INPUT_VID}"
test -n "${OUTPUT_VID}"

./ffmpeg \
    -i "${INPUT_VID}" \
    -vf transform360="input_stereo_format=MONO
    :output_layout=cubemap_23_offcenter
    :w=2048
    :h=3072
    :yaw=-45
    :pitch=-45
    :roll=0
    :expand_coef=1.025
    :interpolation_alg=cubic
    :enable_low_pass_filter=1
    :enable_multi_threading=1
    :width_scale_factor=4
    :height_scale_factor=4
    :num_horizontal_segments=32
    :num_vertical_segments=15
    :adjust_kernel=1" \
    "${OUTPUT_VID}"
