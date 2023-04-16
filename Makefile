debug_server:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/cs32f1x.cfg

debug:
	cargo run

flash:
	cargo flash --chip stm32f103C8
