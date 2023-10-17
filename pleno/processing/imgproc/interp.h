#pragma once

#include "types.h"
#include "transformation.h"

////////////////////////////////////////////////////////////////////////////////
/*
 * Apply transformation
 * TODO: add a last (col,row) in order to remove the black borders
 */
bool warp(const Transformation& t, const Image& input, Image& output);

////////////////////////////////////////////////////////////////////////////////
struct GrayInterpolator {
    co