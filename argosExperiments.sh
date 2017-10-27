#!/bin/bash

for i in {1..100}
do
	argos3 -c experiments/epuck_brownian.argos &>> speedTests_10epuck_nofail.txt
done