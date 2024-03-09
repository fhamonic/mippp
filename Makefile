BUILD_DIR = build

.PHONY: all test clean

all: test

$(BUILD_DIR):
	conan build . -of=${BUILD_DIR} -b=missing

test: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure
	
package:
	conan create . -u -b=missing
	
cross_w64:
	conan build . -of=build_mingw -b=missing -pr:b=default -pr:h=linux_to_win64 -v

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
