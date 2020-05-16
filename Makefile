# Copyright (c) 2020 Eric Bruneton
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

GPP := g++

# Black hole shader demo.

DEMO_SOURCES := \
    $(shell find black_hole/demo/ -type f -name "*" -not -name "demo.html")

demo: output/black_hole/demo/demo.html rocket_model \
    black_hole_textures black_body_texture doppler_texture gaia_sky_map_texture
	@echo "Start an HTTP server in output/ (e.g. with python -m SimpleHTTPServer)"
	@echo "and open http://localhost:8000/black_hole/demo/demo.html"

output/black_hole/demo/demo.html: \
    black_hole/demo/demo.html \
    $(DEMO_SOURCES:%=output/%) \
    output/black_hole/black_hole_shader.glsl \
    output/bloom/bloom.js
	mkdir -p $(@D)
	sed 's/@include/cat/e' $< > $@

output/black_hole/demo/%.html: black_hole/demo/%.html
	mkdir -p $(@D)
	sed -e '/<!DOCTYPE/,/<body>/d' -e '/<\/body>/,/<\/html>/d' \
      -e ':a' -e '/\\$$/N; s/\\\n *//; ta' \
      -e 's/\\n */\n/g' $< > $@

output/black_hole/demo/%: black_hole/demo/%
	mkdir -p $(@D)
	sed -e '/^\/\*/,/\*\/$$/d' $< > $@

# Black hole shader and its unit tests.

black_hole_shader: output/black_hole/black_hole_shader.glsl

output/black_hole/black_hole_shader.glsl: \
    black_hole/definitions.glsl \
    black_hole/functions.glsl \
    black_hole/model.glsl
	mkdir -p $(@D)
	sed -e '/^\/\*/,/\*\/$$/d' black_hole/definitions.glsl > $@
	sed -e '/^\/\*/,/\*\/$$/d' black_hole/functions.glsl >> $@
	sed -e '/^\/\*/,/\*\/$$/d' black_hole/model.glsl >> $@

unit_tests: output/black_hole/preprocess/preprocess_test
	output/black_hole/preprocess/preprocess_test

output/black_hole/preprocess/preprocess_test: \
    output/black_hole/preprocess/functions.o \
    output/black_hole/preprocess/functions_test.o \
    output/submodules/dimensional_types/test/test_main.o
	$(GPP) $^ -o $@

# Black hole ray deflection and ray inverse radius textures generator.

black_hole_textures: output/black_hole/demo/deflection.dat

output/black_hole/demo/deflection.dat: \
    output/black_hole/preprocess/preprocess
	mkdir -p $(@D)
	output/black_hole/preprocess/preprocess output/black_hole/demo/

output/black_hole/preprocess/preprocess: \
    output/black_hole/preprocess/functions.o \
    output/black_hole/preprocess/preprocess_main.o
	$(GPP) $^ -o $@

output/black_hole/preprocess/*.o: \
    black_hole/preprocess/definitions.h \
    black_hole/functions.glsl

# Bloom shader and bloom filters generator.

bloom_shader: output/bloom/bloom.js

output/bloom/bloom.js: bloom/bloom.js output/bloom/filters.js
	mkdir -p $(@D)
	cat output/bloom/filters.js > $@
	sed -e '/^\/\*/,/\*\/$$/d' $< >> $@

output/bloom/filters.js: output/bloom/bloom_filters_generator
	mkdir -p $(@D)
	output/bloom/bloom_filters_generator $@

output/bloom/bloom_filters_generator: output/bloom/bloom_filters_generator.o
	$(GPP) $^ -o $@

# Rocket 3D model and model generator.

rocket_model: output/black_hole/demo/rocket.dat

output/black_hole/demo/rocket.dat: output/rocket/rocket_generator \
    rocket/color.png \
    rocket/height.png \
    rocket/occlusion.png \
    rocket/roughness.png \
    rocket/metallic.png
	mkdir -p $(@D)
	output/rocket/rocket_generator rocket/ output/black_hole/demo/

output/rocket/rocket_generator: output/rocket/rocket_generator.o
	$(GPP) $^ -lpng -o $@

# Gaia sky map generator.

ESA_IMAGES := https://www.esa.int/var/esa/storage/images/esa_multimedia/images
GAIA_SKY := ${ESA_IMAGES}/2018/04/gaia_s_sky_in_colour2
GAIA_PNG := ${GAIA_SKY}/17475369-5-eng-GB/Gaia_s_sky_in_colour.png
GAIA_INDEX := http://cdn.gea.esac.esa.int/Gaia/gdr2/gaia_source/csv/
TYCHO_BASE := http://cdsarc.u-strasbg.fr/ftp/I/259

gaia_sky_map_texture: output/gaia_sky_map/pos-x-0-0-0.dat

output/gaia_sky_map/pos-x-0-0-0.dat: \
    output/gaia_sky_map/gaia_sky_map_generator \
    output/gaia_sky_map/gaia.png \
    output/gaia_sky_map/gaia.html \
    output/gaia_sky_map/tyc2.dat \
    output/gaia_sky_map/suppl_1.dat
	mkdir -p output/gaia_sky_map
	output/gaia_sky_map/gaia_sky_map_generator output/gaia_sky_map/

output/gaia_sky_map/gaia_sky_map_generator: \
    output/submodules/rgb9_e5/rgb9_e5.o \
    output/color_maps/colors.o \
    output/gaia_sky_map/colors.o \
    output/gaia_sky_map/cube_map.o \
    output/gaia_sky_map/gaia.o \
    output/gaia_sky_map/gaia_color.o \
    output/gaia_sky_map/gaia_sky_map_generator.o \
    output/gaia_sky_map/tycho.o
	$(GPP) $^ -lcurl -lcurlpp -lpng -lpthread -lz -o $@

output/gaia_sky_map/gaia.png:
	mkdir -p $(@D)
	curl -o $@ ${GAIA_PNG}

output/gaia_sky_map/gaia.html:
	mkdir -p $(@D)
	curl -o $@ ${GAIA_INDEX}

output/gaia_sky_map/tyc2.dat:
	mkdir -p $(@D)
	curl -o - ${TYCHO_BASE}/tyc2.dat.00.gz | gunzip > $@
	for i in 01 02 03 04 05 06 07 08 09 10 11 12 13 14 15 16 17 18 19 ; do \
    curl -o - ${TYCHO_BASE}/tyc2.dat.$$i.gz | gunzip >> $@ ; \
  done

output/gaia_sky_map/suppl_1.dat:
	mkdir -p $(@D)
	curl -o - ${TYCHO_BASE}/suppl_1.dat.gz | gunzip > $@

# Black-body and Doppler textures generator.

black_body_texture: output/black_hole/demo/black_body.dat
doppler_texture: output/black_hole/demo/doppler.dat

output/black_hole/demo/black_body.dat: \
    output/color_maps/black_body_texture_generator
	mkdir -p output/black_hole/demo
	output/color_maps/black_body_texture_generator $@

output/black_hole/demo/doppler.dat: \
    output/color_maps/doppler_texture_generator
	mkdir -p output/black_hole/demo
	output/color_maps/doppler_texture_generator $@

output/color_maps/black_body_texture_generator: \
    output/color_maps/colors.o \
    output/color_maps/black_body_texture_generator.o
	$(GPP) $^ -o $@

output/color_maps/doppler_texture_generator: \
    output/color_maps/colors.o \
    output/color_maps/doppler_texture_generator.o
	$(GPP) $^ -o $@

# Compilation.

GPP_FLAGS := -Wall -Wmain -pedantic -pedantic-errors -std=c++11 -DNDEBUG \
    -O3 -fexpensive-optimizations
INCLUDE_FLAGS := -I. -Isubmodules -Isubmodules/dimensional_types \
    -Isubmodules/eigen3_nnls/src -Isubmodules/gzip/include

output/%.o: %.cc
	mkdir -p $(@D)
	$(GPP) $(GPP_FLAGS) $(INCLUDE_FLAGS) -c $< -o $@

