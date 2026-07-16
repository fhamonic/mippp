BUILD_DIR = build

CONAN_PROFILE = gcc14_c++23
# CONAN_PROFILE = gcc15_c++26
# CONAN_PROFILE = debug_c++26

ifeq (test,$(firstword $(MAKECMDGOALS)))
	ifeq ($(words $(MAKECMDGOALS)),2)
		TEST_SOURCE := $(word 2,$(MAKECMDGOALS))
		$(eval $(TEST_SOURCE):;@:)
	else ifeq ($(words $(MAKECMDGOALS)),3)
		TEST_SOURCE := $(word 2,$(MAKECMDGOALS))
		TEST_FILTER := $(word 3,$(MAKECMDGOALS))
		$(eval $(TEST_SOURCE):;@:)
		$(eval $(TEST_FILTER):;@:)
	endif
endif

.PHONY: all test package clean

all: test

test:
	TEST_SOURCE="$(TEST_SOURCE)" TEST_FILTER="$(TEST_FILTER)" conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE}
	
package:
	conan create . -u -b=missing -pr=${CONAN_PROFILE} -c tools.build:skip_test=true

doc:
	python docs/assets/features_tables/tested_features_table.py
	zensical serve

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
