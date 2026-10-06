cmake_minimum_required(VERSION 3.25)
project(elNoise VERSION 1.2.0 LANGUAGES C CXX)
enable_language(RC)

# --- Steinberg VST3 SDK (git submodule, MIT) -----------------------------------------------
set(SMTG_ENABLE_VST3_PLUGIN_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SMTG_ENABLE_VST3_HOSTING_EXAMPLES OFF CACHE BOOL "" FORCE)
set(SMTG_ENABLE_VSTGUI_SUPPORT OFF CACHE BOOL "" FORCE)
set(SMTG_CREATE_PLUGIN_LINK OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/vst3sdk ${CMAKE_BINARY_DIR}/vst3sdk)

# --- SpeexDSP (git submodule, BSD-3) ---------------------------------------------------------
set(SPEEX ${CMAKE_CURRENT_SOURCE_DIR}/external/speexdsp)
add_library(speexdsp_pp STATIC
    ${SPEEX}/libspeexdsp/preprocess.c
    ${SPEEX}/libspeexdsp/fftwrap.c
    ${SPEEX}/libspeexdsp/filterbank.c
    ${SPEEX}/libspeexdsp/smallft.c
    ${SPEEX}/libspeexdsp/mdf.c)
target_include_directories(speexdsp_pp
    PUBLIC ${SPEEX}/include ${CMAKE_CURRENT_SOURCE_DIR}/gen
    PRIVATE ${SPEEX}/libspeexdsp ${SPEEX}/win32)
target_compile_definitions(speexdsp_pp PRIVATE HAVE_CONFIG_H EXPORT=)
set_target_properties(speexdsp_pp PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- RNNoise v0.2 (git submodule, BSD-3) ----------------------------------------------------------
# The two trained models (full + little, ~22 MB archive) are not part of the git repository; they are
# downloaded once at configure time and verified against a pinned SHA-256.
set(RNN ${CMAKE_CURRENT_SOURCE_DIR}/external/rnnoise)
set(RNN_GEN ${CMAKE_BINARY_DIR}/rnnoise_model)
set(RNN_MODEL_URL "https://media.xiph.org/rnnoise/models/rnnoise_data-0b50c45.tar.gz")
set(RNN_MODEL_SHA256 "4ac81c5c0884ec4bd5907026aaae16209b7b76cd9d7f71af582094a2f98f4b43")
if(NOT EXISTS ${RNN_GEN}/src/rnnoise_data.c)
    file(MAKE_DIRECTORY ${RNN_GEN})
    message(STATUS "Downloading RNNoise models ...")
    file(DOWNLOAD ${RNN_MODEL_URL} ${RNN_GEN}/model.tar.gz EXPECTED_HASH SHA256=${RNN_MODEL_SHA256} SHOW_PROGRESS)
    file(ARCHIVE_EXTRACT INPUT ${RNN_GEN}/model.tar.gz DESTINATION ${RNN_GEN}
         PATTERNS src/rnnoise_data.c src/rnnoise_data.h src/rnnoise_data_little.c src/rnnoise_data_little.h)
endif()

add_library(rnnoise_lib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rnnoise_wrapper.c   # includes denoise.c + the "light" constructor
    ${RNN}/src/rnn.c ${RNN}/src/pitch.c ${RNN}/src/kiss_fft.c ${RNN}/src/celt_lpc.c ${RNN}/src/nnet.c
    ${RNN}/src/nnet_default.c ${RNN}/src/parse_lpcnet_weights.c ${RNN}/src/rnnoise_tables.c
    ${RNN_GEN}/src/rnnoise_data.c ${RNN_GEN}/src/rnnoise_data_little.c)
# the little model reuses the same symbol names, so rename them for that one file
set_source_files_properties(${RNN_GEN}/src/rnnoise_data_little.c PROPERTIES
    COMPILE_DEFINITIONS "rnnoise_arrays=rnnoise_arrays_light;init_rnnoise=init_rnnoise_light")
target_include_directories(rnnoise_lib
    PUBLIC ${RNN}/include
    PRIVATE ${RNN}/src ${RNN_GEN}/src ${CMAKE_CURRENT_SOURCE_DIR}/gen/rnnoise)
target_compile_definitions(rnnoise_lib PRIVATE _USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
set_target_properties(rnnoise_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# --- the plugin --------------------------------------------------------------------------------
smtg_add_vst3plugin(elNoise
    src/plugin.cpp
    assets/elnoise.rc
    external/vst3sdk/public.sdk/source/vst/vstsinglecomponenteffect.cpp)
target_link_libraries(elNoise PRIVATE sdk speexdsp_pp rnnoise_lib)
target_compile_features(elNoise PRIVATE cxx_std_17)
