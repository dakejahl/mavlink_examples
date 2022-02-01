all:
	cmake -Bbuild -H.; cmake --build build

camera:
	cmake -Bbuild -H.; cmake --build build --target camera

receive:
	cmake -Bbuild -H.; cmake --build build --target receive

stress_test:
	cmake -Bbuild -H.; cmake --build build --target stress_test

winch:
	cmake -Bbuild -H.; cmake --build build --target winch

gimbal_control:
	cmake -Bbuild -H.; cmake --build build --target gimbal_control

clean:
	@rm -rf build/
	@echo "All build artifacts removed"

.PHONY: all clean gimbal_control