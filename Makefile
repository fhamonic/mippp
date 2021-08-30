MAKEFLAGS += --no-print-directory

CPUS?=$(shell getconf _NPROCESSORS_ONLN || echo 1)

BUILD_DIR = build

.PHONY: all clean single-header doc

all: $(BUILD_DIR)
	@cd $(BUILD_DIR) && \
	cmake --build . --parallel $(CPUS)

$(BUILD_DIR):
	@mkdir $(BUILD_DIR) && \
	cd $(BUILD_DIR) && \
	conan install .. && \
	cmake -DCMAKE_CXX_COMPILER=g++-10 -DCMAKE_BUILD_TYPE=Release -DWARNINGS=ON -DHARDCORE_WARNINGS=ON -DCOMPILE_FOR_NATIVE=ON -DCOMPILE_WITH_LTO=ON ..

clean:
	@rm -rf $(BUILD_DIR)

single-header: single-header/milppp.hpp

single-header/milppp.hpp:
	@python3 -m quom --include_directory include include/milppp.hpp milppp.hpp.tmp && \
	mkdir -p single-header && \
	echo "/*" > single-header/milppp.hpp && \
	cat LICENSE >> single-header/milppp.hpp && \
	echo "*/" >> single-header/milppp.hpp && \
	cat milppp.hpp.tmp >> single-header/milppp.hpp && \
	rm milppp.hpp.tmp

doc:
	doxywizard $$PWD/doc/Doxyfile
	xdg-open doc/html/index.html 