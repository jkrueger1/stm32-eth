BIN = mesygen

.PHONY: build flash setup

build:
	~/.cargo/bin/cargo build --release --bin=$(BIN)

flash: build
	@(openocd -f /usr/share/openocd/scripts/board/st_nucleo_f4.cfg & \
	 OCD=$$! ; \
	 sleep 1 ; \
	 arm-none-eabi-gdb -q -nx -ex "target remote :3333" -ex load -ex detach -ex quit target/thumbv7em-none-eabihf/release/$(BIN) ; \
	 kill $$OCD ; \
	 echo Flashing done.)

setup:
	sudo apt-get install curl build-essential gdb-arm-none-eabi openocd
	curl https://sh.rustup.rs -sSf | sh -s -- -y --no-modify-path --default-toolchain nightly-2018-08-06
	~/.cargo/bin/rustup target add thumbv7em-none-eabihf
	~/.cargo/bin/cargo generate-lockfile
	@echo Setup done.
