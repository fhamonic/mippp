BUILD_DIR = build
CONAN_PROFILE = debug
# CONAN_PROFILE = default_c++23

.PHONY: all test clean build

all: test

$(BUILD_DIR):
	conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE}

test: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure
	
package:
	conan create . -u -b=missing -pr=${CONAN_PROFILE}

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
