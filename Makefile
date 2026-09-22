BUILD_DIR = build

CONAN_PROFILE = gcc14_c++23
# CONAN_PROFILE = gcc14_c++23_debug
# CONAN_PROFILE = gcc15_c++26
# CONAN_PROFILE = gcc15_c++26_debug
# CONAN_PROFILE = clang18_c++23

CONAN_CXXFLAGS = -c 'tools.build:cxxflags=["-fconcepts-diagnostics-depth=30"]'

.PHONY: all test examples package check-format check-includes check features_tables compat_table doc paper clean

all: test

# Ignore unknown targets:
%:
	@:

ifeq ($(firstword $(MAKECMDGOALS)),test)
    TEST_SOURCE := $(word 2,$(MAKECMDGOALS))
    TEST_FILTER := $(word 3,$(MAKECMDGOALS))
endif

test:
	TEST_SOURCE="$(TEST_SOURCE)" TEST_FILTER="$(TEST_FILTER)" conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE} ${CONAN_CXXFLAGS}
	
examples:
	ENABLE_EXAMPLES=ON conan build . -of=${BUILD_DIR} -b=missing -pr=${CONAN_PROFILE} -c tools.build:skip_test=true

package:
	conan create . -u -b=missing -pr=${CONAN_PROFILE} -c tools.build:skip_test=true

check-format:
	find include test -name "*.hpp" -o -name "*.cpp" | xargs clang-format --dry-run -Werror

check-includes:
	python3 misc/tools/check_std_includes.py include

check: check-format check-includes

features_tables:
	python docs/assets/features_tables/tested_features_table.py

compat_table:
	python3 misc/tools/compat_matrix.py run --limit $(or $(LIMIT),10) --commercial

doc:
	zensical serve

paper:
	@rm -f paper/paper.pdf
	docker run --rm -v "$(CURDIR):/data" -u $(shell id -u):$(shell id -g) openjournals/inara -o pdf paper/paper.md

clean:
	@rm -rf CMakeUserPresets.json
	@rm -rf $(BUILD_DIR)
	@rm -rf .compat-cache
	@rm -f paper/paper.pdf
