BUILD_DIR = build

CONAN_PROFILE = default_c++26
# CONAN_PROFILE = debug_c++26

ifeq (test,$(firstword $(MAKECMDGOALS)))
  TEST_FILTER := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
  $(eval $(TEST_FILTER):;@:)
endif

.PHONY: all test package clean

all: test

test:
	TEST_FILTER="$(TEST_FILTER)" conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE}
	
package:
	conan create . -u -b=missing -pr=${CONAN_PROFILE} -c tools.build:skip_test=true

doc:
	zensical serve

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
