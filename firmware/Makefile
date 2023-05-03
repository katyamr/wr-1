all: firmware
.PHONY: firmware

firmware:
	arduino-cli -v compile -b arduino:avr:leonardo --libraries ~/.arduino15/libraries --libraries ~/Library/Arduino15/libraries --warnings all .

upload: firmware
	arduino-cli -v upload -b arduino:avr:leonardo -p $(PORT) .
