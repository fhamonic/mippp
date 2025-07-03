BUILD_DIR = build
# CONAN_PROFILE = debug
CONAN_PROFILE = default_c++26

.PHONY: all test package clean

all: test

test:
	conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE}
	
package:
	conan create . -u -b=missing -pr=${CONAN_PROFILE} -c tools.build:skip_test=true

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
