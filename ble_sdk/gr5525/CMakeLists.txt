# Copyright (c) 2024 Goodix
# SPDX-License-Identifier: Apache-2.0

# GR5525 SDK CMake wrapper

set(SDK_FOLDER "GR5525.SDK")
set(SDK_MBEDTLS_ROOT "${SDK_FOLDER}/external/mbedtls/mbedtls-2.26.0_hw")

zephyr_include_directories(
    ${SDK_FOLDER}/build/config
    ${SDK_FOLDER}/components/libraries/app_memory
    ${SDK_FOLDER}/components/libraries/crypto_lib/inc
    ${SDK_FOLDER}/components/libraries/hal_flash
    ${SDK_FOLDER}/components/libraries/utility
    ${SDK_FOLDER}/components/sdk
    ${SDK_FOLDER}/drivers/inc
    ${SDK_FOLDER}/drivers/inc/hal
    ${SDK_FOLDER}/platform/arch/arm/cortex-m/cmsis/core/include
    ${SDK_FOLDER}/platform/include
    ${SDK_FOLDER}/platform/soc/include
)

if (CONFIG_FP_HARDABI OR FORCE_FP_HARDABI)
    set(SDK_LIB_PREFIX "hardfloat_lib/")
endif()

# Use absolute path for library
zephyr_library_link_libraries(
    ${CMAKE_CURRENT_SOURCE_DIR}/${SDK_FOLDER}/platform/soc/linker/gcc/${SDK_LIB_PREFIX}libble_sdk.a
    ${CMAKE_CURRENT_SOURCE_DIR}/${SDK_FOLDER}/platform/soc/linker/gcc/rom_symbol_gcc.txt
)

zephyr_sources_ifdef(
    CONFIG_BT_GR5525
    ${SDK_FOLDER}/components/sdk/ble.c
)

zephyr_sources_ifdef(
    CONFIG_GOODIX_APP_MEMORY_LIB
    ${SDK_FOLDER}/components/libraries/app_memory/app_memory.c
)

zephyr_sources_ifdef(
    CONFIG_GOODIX_HW_SE_LIB
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_aes.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_ecc.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_ecc_port.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_gcm.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_pkc.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_pkc_port.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_rsa.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_rsa_port.c
    ${SDK_FOLDER}/components/libraries/crypto_lib/src/crypto_sha256.c
)

zephyr_include_directories_ifdef(
    CONFIG_GOODIX_OPT_MBEDTLS_LIB
    ${SDK_MBEDTLS_ROOT}/configs
    ${SDK_MBEDTLS_ROOT}/include
    ${SDK_MBEDTLS_ROOT}/include/mbedtls
)

zephyr_sources_ifdef(
    CONFIG_GOODIX_OPT_MBEDTLS_LIB
    ${SDK_MBEDTLS_ROOT}/library/aes.c
    ${SDK_MBEDTLS_ROOT}/library/aesni.c
    ${SDK_MBEDTLS_ROOT}/library/arc4.c
    ${SDK_MBEDTLS_ROOT}/library/aria.c
    ${SDK_MBEDTLS_ROOT}/library/asn1parse.c
    ${SDK_MBEDTLS_ROOT}/library/asn1write.c
    ${SDK_MBEDTLS_ROOT}/library/base64.c
    ${SDK_MBEDTLS_ROOT}/library/bignum.c
    ${SDK_MBEDTLS_ROOT}/library/blowfish.c
    ${SDK_MBEDTLS_ROOT}/library/camellia.c
    ${SDK_MBEDTLS_ROOT}/library/ccm.c
    ${SDK_MBEDTLS_ROOT}/library/certs.c
    ${SDK_MBEDTLS_ROOT}/library/chacha20.c
    ${SDK_MBEDTLS_ROOT}/library/chachapoly.c
    ${SDK_MBEDTLS_ROOT}/library/cipher.c
    ${SDK_MBEDTLS_ROOT}/library/cipher_wrap.c
    ${SDK_MBEDTLS_ROOT}/library/cmac.c
    ${SDK_MBEDTLS_ROOT}/library/ctr_drbg.c
    ${SDK_MBEDTLS_ROOT}/library/des.c
    ${SDK_MBEDTLS_ROOT}/library/dhm.c
    ${SDK_MBEDTLS_ROOT}/library/ecdh.c
    ${SDK_MBEDTLS_ROOT}/library/ecdsa.c
    ${SDK_MBEDTLS_ROOT}/library/ecjpake.c
    ${SDK_MBEDTLS_ROOT}/library/ecp.c
    ${SDK_MBEDTLS_ROOT}/library/ecp_curves.c
    ${SDK_MBEDTLS_ROOT}/library/entropy.c
    ${SDK_MBEDTLS_ROOT}/library/entropy_poll.c
    ${SDK_MBEDTLS_ROOT}/library/error.c
    ${SDK_MBEDTLS_ROOT}/library/gcm.c
    ${SDK_MBEDTLS_ROOT}/library/havege.c
    ${SDK_MBEDTLS_ROOT}/library/hkdf.c
    ${SDK_MBEDTLS_ROOT}/library/hmac_drbg.c
    ${SDK_MBEDTLS_ROOT}/library/md.c
    ${SDK_MBEDTLS_ROOT}/library/md2.c
    ${SDK_MBEDTLS_ROOT}/library/md4.c
    ${SDK_MBEDTLS_ROOT}/library/md5.c
    ${SDK_MBEDTLS_ROOT}/library/memory_buffer_alloc.c
    ${SDK_MBEDTLS_ROOT}/library/net_sockets.c
    ${SDK_MBEDTLS_ROOT}/library/nist_kw.c
    ${SDK_MBEDTLS_ROOT}/library/oid.c
    ${SDK_MBEDTLS_ROOT}/library/padlock.c
    ${SDK_MBEDTLS_ROOT}/library/pkcs11.c
    ${SDK_MBEDTLS_ROOT}/library/pkcs12.c
    ${SDK_MBEDTLS_ROOT}/library/pkcs5.c
    ${SDK_MBEDTLS_ROOT}/library/pkparse.c
    ${SDK_MBEDTLS_ROOT}/library/pkwrite.c
    ${SDK_MBEDTLS_ROOT}/library/platform.c
    ${SDK_MBEDTLS_ROOT}/library/platform_util.c
    ${SDK_MBEDTLS_ROOT}/library/poly1305.c
    ${SDK_MBEDTLS_ROOT}/library/ripemd160.c
    ${SDK_MBEDTLS_ROOT}/library/rsa.c
    ${SDK_MBEDTLS_ROOT}/library/rsa_internal.c
    ${SDK_MBEDTLS_ROOT}/library/sha1.c
    ${SDK_MBEDTLS_ROOT}/library/sha256.c
    ${SDK_MBEDTLS_ROOT}/library/sha512.c
    ${SDK_MBEDTLS_ROOT}/library/threading.c
    ${SDK_MBEDTLS_ROOT}/library/timing.c
    ${SDK_MBEDTLS_ROOT}/library/version.c
    ${SDK_MBEDTLS_ROOT}/library/version_features.c
    ${SDK_MBEDTLS_ROOT}/library/xtea.c
)