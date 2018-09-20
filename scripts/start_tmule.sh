#!/usr/bin/env bash
export TMULE_CONFIG=$1

cd $TMULE_CONFIG

tmule --config rob_lindsey.yaml server
