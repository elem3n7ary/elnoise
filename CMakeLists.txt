cmake_minimum_required(VERSION 3.25)
project(SpeexNoiseSuppression VERSION 1.0.0 LANGUAGES C CXX)

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

# --- the plugin --------------------------------------------------------------------------------
smtg_add_vst3plugin(SpeexNoiseSuppression
    src/plugin.cpp
    external/vst3sdk/public.sdk/source/vst/vstsinglecomponenteffect.cpp)
target_link_libraries(SpeexNoiseSuppression PRIVATE sdk speexdsp_pp)
target_compile_features(SpeexNoiseSuppression PRIVATE cxx_std_17)
