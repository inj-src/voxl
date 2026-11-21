# voxl overlay of vcpkg's x264 port (microsoft/vcpkg 114d9fe62faf, ports/x264).
# The only change is the voxl patches at the end of PATCHES: they skip the
# macroblocks voxl marks unchanged (see x264.nix). They are read from
# patches/x264 at the repository root, so an update there reaches this build.
# The patches were checked against this ref. If you move it, check them again.

# The latest ref in branch stable
set(ref b35605ace3ddf7c1a5d67a2eb553f034aef41d55)

# Note on x264 versioning:
# The pc file exports "0.<ABI>.<N>" where N is the number of commits.
# The binary releases on https://artifacts.videolan.org/x264/ are named x264-r<N>-<COMMIT>.
# With a git clone, this can be determined by running `versions.sh`.
# With vcpkg_from_gitlab, we modify `versions.sh` accordingly.
# For --editable mode, use configured patch instead of vcpkg_replace_string.
string(REGEX MATCH "^......." short_ref "${ref}")
string(REGEX MATCH "[0-9]+\$" revision "${VERSION}")
configure_file("${CURRENT_PORT_DIR}/version.diff.in" "${CURRENT_BUILDTREES_DIR}/src/version-${VERSION}.diff" @ONLY)

vcpkg_from_gitlab(
    GITLAB_URL https://code.videolan.org
    OUT_SOURCE_PATH SOURCE_PATH
    REPO videolan/x264
    REF "${ref}"
    SHA512 bfac118da55da2fcc4587b17a3184d9ed70d6e03188bc0497f5922df22b5685fa49fa4325f9c5196c76d03e6e3f56d5906475c540d0df7e1739dc4182816eebe
    HEAD_REF master
    PATCHES
        "${CURRENT_BUILDTREES_DIR}/src/version-${VERSION}.diff"
        uwp-cflags.patch
        parallel-install.patch
        allow-clang-cl.patch
        configure.patch
        ../../../patches/x264/0001-Skip-constant-macroblocks-without-analysing-them.patch
        ../../../patches/x264/0002-Copy-only-the-input-macroblocks-that-changed.patch
        ../../../patches/x264/0003-Filter-only-the-lowres-blocks-whose-input-changed.patch
        ../../../patches/x264/0004-Copy-large-changes-whole-and-changed-luma-with-the-S.patch
        ../../../patches/x264/0005-Skip-constant-macroblocks-only-where-the-full-path-would-at-emergency-QPs.patch
        ../../../patches/x264/0006-Align-changed-luma-runs-to-the-AVX-512-copy-s-64-bytes.patch
)

function(add_cross_prefix)
  if(configure_env MATCHES "CC=([^\/]*-)gcc$")
      vcpkg_list(APPEND arg_OPTIONS "--cross-prefix=${CMAKE_MATCH_1}")
  endif()
  set(arg_OPTIONS "${arg_OPTIONS}" PARENT_SCOPE)
endfunction()

set(nasm_archs x86 x64)
set(gaspp_archs arm arm64)
if(NOT "asm" IN_LIST FEATURES)
    vcpkg_list(APPEND OPTIONS --disable-asm)
elseif(NOT "$ENV{AS}" STREQUAL "")
    # Accept setting from triplet
elseif(VCPKG_TARGET_ARCHITECTURE IN_LIST nasm_archs)
    vcpkg_find_acquire_program(NASM)
    vcpkg_insert_program_into_path("${NASM}")
    set(ENV{AS} "${NASM}")
elseif(VCPKG_TARGET_ARCHITECTURE IN_LIST gaspp_archs AND VCPKG_TARGET_IS_WINDOWS AND VCPKG_HOST_IS_WINDOWS)
    vcpkg_find_acquire_program(GASPREPROCESSOR)
    list(FILTER GASPREPROCESSOR INCLUDE REGEX gas-preprocessor)
    file(INSTALL "${GASPREPROCESSOR}" DESTINATION "${SOURCE_PATH}/tools" RENAME "gas-preprocessor.pl")
endif()

vcpkg_list(SET OPTIONS_RELEASE)
if("tool" IN_LIST FEATURES)
    vcpkg_list(APPEND OPTIONS_RELEASE --enable-cli)
else()
    vcpkg_list(APPEND OPTIONS_RELEASE --disable-cli)
endif()

if("chroma-format-all" IN_LIST FEATURES)
    vcpkg_list(APPEND OPTIONS --chroma-format=all)
endif()

if(NOT "gpl" IN_LIST FEATURES)
    vcpkg_list(APPEND OPTIONS --disable-gpl)
endif()

if(VCPKG_TARGET_IS_UWP)
    list(APPEND OPTIONS --extra-cflags=-D_WIN32_WINNT=0x0A00)
endif()

vcpkg_make_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    DISABLE_CPPFLAGS # Build is not using CPP/CPPFLAGS
    DISABLE_MSVC_WRAPPERS
    LANGUAGES ASM C CXX # Requires NASM to compile
    DISABLE_MSVC_TRANSFORMATIONS # disable warnings about unknown -Xcompiler/-Xlinker flags
    PRE_CONFIGURE_CMAKE_COMMANDS
        add_cross_prefix
    OPTIONS
        ${OPTIONS}
        --enable-pic
        --disable-lavf
        --disable-swscale
        --disable-avs
        --disable-ffms
        --disable-gpac
        --disable-lsmash
        --disable-bashcompletion
    OPTIONS_RELEASE
        ${OPTIONS_RELEASE}
        --enable-strip
        "--bindir=\\\${prefix}/bin"
    OPTIONS_DEBUG
        --enable-debug
        --disable-cli
        "--bindir=\\\${prefix}/bin"
)

vcpkg_make_install()

if("tool" IN_LIST FEATURES)
    vcpkg_copy_tools(TOOL_NAMES x264 AUTO_CLEAN)
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")

if(VCPKG_TARGET_IS_WINDOWS AND NOT VCPKG_TARGET_IS_MINGW)
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/lib/pkgconfig/x264.pc" "-lx264" "-llibx264")
    if(NOT VCPKG_BUILD_TYPE)
        vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/debug/lib/pkgconfig/x264.pc" "-lx264" "-llibx264")
    endif()
endif()

if(VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic" AND VCPKG_TARGET_IS_WINDOWS AND NOT VCPKG_TARGET_IS_MINGW)
    file(RENAME "${CURRENT_PACKAGES_DIR}/lib/libx264.dll.lib" "${CURRENT_PACKAGES_DIR}/lib/libx264.lib")
    if (NOT VCPKG_BUILD_TYPE)
        file(RENAME "${CURRENT_PACKAGES_DIR}/debug/lib/libx264.dll.lib" "${CURRENT_PACKAGES_DIR}/debug/lib/libx264.lib")
    endif()
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/x264.h" "#ifdef X264_API_IMPORTS" "#if 1")
elseif(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/x264.h" "defined(U_STATIC_IMPLEMENTATION)" "1" IGNORE_UNCHANGED)
    file(REMOVE_RECURSE
        "${CURRENT_PACKAGES_DIR}/bin"
        "${CURRENT_PACKAGES_DIR}/debug/bin"
    )
endif()

vcpkg_fixup_pkgconfig()

vcpkg_copy_pdbs()

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/COPYING")
