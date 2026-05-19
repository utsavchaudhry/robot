#!/bin/bash

$branch = $1

git switch $branch
git submodule update --remote --recursive
