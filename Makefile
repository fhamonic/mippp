MAKEFLAGS += --no-print-directory

CPUS?=$(shell getconf _NPROCESSORS_ONLN || echo 1)

BUILD_DIR = build

.PHONY: all test clean

all: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cmake --build . --parallel $(CPUS)

$(BUILD_DIR):
	@mkdir $(BUILD_DIR) && \
	cd $(BUILD_DIR) && \
	cmake -DCMAKE_BUILD_TYPE=Debug -DENABLE_TESTING=ON ..

test: all
	@cd $(BUILD_DIR) && \
	ctest --output-on-failure

clean:
	@rm -rf $(BUILD_DIR)

