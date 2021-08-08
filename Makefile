MAKEFLAGS += --no-print-directory

CPUS?=$(shell getconf _NPROCESSORS_ONLN || echo 1)

BUILD_DIR = build

.PHONY: all clean doc

all: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cmake --build . --parallel $(CPUS)

$(BUILD_DIR):
	@mkdir $(BUILD_DIR) && \
	cd $(BUILD_DIR) && \
	conan install .. && \
	cmake -DCMAKE_CXX_COMPILER=g++-10 -DCMAKE_BUILD_TYPE=Release -DWARNINGS=ON -DCOMPILE_FOR_NATIVE=ON -DCOMPILE_WITH_LTO=ON ..

clean:
	@rm -rf $(BUILD_DIR)

doc:
	doxywizard $$PWD/doc/Doxyfile
	xdg-open doc/html/index.html 