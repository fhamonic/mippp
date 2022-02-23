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

single-header: single-header/mippp_cbc.hpp single-header/mippp_grb.hpp

single-header/mippp_cbc.hpp:
	@python3 -m quom --include_directory include include/mippp_cbc.hpp mippp_cbc.hpp.tmp && \
	mkdir -p single-header && \
	echo "/*" > single-header/mippp_cbc.hpp && \
	cat LICENSE >> single-header/mippp_cbc.hpp && \
	echo "*/" >> single-header/mippp_cbc.hpp && \
	cat mippp_cbc.hpp.tmp >> single-header/mippp_cbc.hpp && \
	rm mippp_cbc.hpp.tmp

single-header/mippp_grb.hpp:
	@python3 -m quom --include_directory include include/mippp_grb.hpp mippp_grb.hpp.tmp && \
	mkdir -p single-header && \
	echo "/*" > single-header/mippp_grb.hpp && \
	cat LICENSE >> single-header/mippp_grb.hpp && \
	echo "*/" >> single-header/mippp_grb.hpp && \
	cat mippp_grb.hpp.tmp >> single-header/mippp_grb.hpp && \
	rm mippp_grb.hpp.tmp
