debug_server:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/cs32f1x.cfg

debug:
	cargo run

build:
	cargo build

flash:
	cargo flash --chip stm32f103C8

detect:
	probe-rs-cli info
