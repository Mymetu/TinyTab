set(MESHCORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/upstream/src")
set(COMPANION_SRC "${CMAKE_CURRENT_SOURCE_DIR}/upstream/companion_radio")
set(CRYPTO_SRC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/Crypto")
set(ED25519_SRC "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ed25519")

file(GLOB ED25519_SOURCES "${ED25519_SRC}/*.c")

idf_component_register(
    SRCS
        "meshcore_core.cpp"
        "port/arduino_compat.cpp"
        "${MESHCORE_SRC}/Dispatcher.cpp"
        "${MESHCORE_SRC}/Identity.cpp"
        "${MESHCORE_SRC}/Mesh.cpp"
        "${MESHCORE_SRC}/Packet.cpp"
        "${MESHCORE_SRC}/Utils.cpp"
        "${MESHCORE_SRC}/helpers/AdvertDataHelpers.cpp"
        "${MESHCORE_SRC}/helpers/BaseChatMesh.cpp"
        "${MESHCORE_SRC}/helpers/IdentityStore.cpp"
        "${MESHCORE_SRC}/helpers/StaticPoolPacketManager.cpp"
        "${MESHCORE_SRC}/helpers/TransportKeyStore.cpp"
        "${MESHCORE_SRC}/helpers/TxtDataHelpers.cpp"
        "${COMPANION_SRC}/DataStore.cpp"
        "${COMPANION_SRC}/MyMesh.cpp"
        "third_party/CayenneLPP/src/CayenneLPP.cpp"
        "third_party/CayenneLPP/src/CayenneLPPPolyline.cpp"
        "${CRYPTO_SRC}/AES128.cpp"
        "${CRYPTO_SRC}/AESCommon.cpp"
        "${CRYPTO_SRC}/BigNumberUtil.cpp"
        "${CRYPTO_SRC}/BlockCipher.cpp"
        "${CRYPTO_SRC}/Crypto.cpp"
        "${CRYPTO_SRC}/Curve25519.cpp"
        "${CRYPTO_SRC}/Ed25519.cpp"
        "${CRYPTO_SRC}/Hash.cpp"
        "${CRYPTO_SRC}/NoiseSource.cpp"
        "${CRYPTO_SRC}/RNG.cpp"
        "${CRYPTO_SRC}/SHA256.cpp"
        "${CRYPTO_SRC}/SHA512.cpp"
        ${ED25519_SOURCES}
    INCLUDE_DIRS
        "include"
        "port/include"
        "upstream/src"
        "upstream/companion_radio"
        "third_party/base64/src"
        "third_party/CayenneLPP/src"
        "third_party/Crypto"
        "third_party/ed25519"
    REQUIRES
        meshcore_radio
        esp_system
        esp_timer
        nvs_flash
        spiffs
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    ESP32=1
    USE_SX1262=1
    USE_SX1268=1
    LORA_FREQ=480.375
    LORA_BW=125
    LORA_SF=11
    LORA_CR=5
    LORA_TX_POWER=1
    MAX_LORA_TX_POWER=22
    SX126X_RX_BOOSTED_GAIN=1
    MAX_CONTACTS=350
    MAX_GROUP_CHANNELS=40
    OFFLINE_QUEUE_SIZE=256
    BLE_PIN_CODE=123456
    ENV_INCLUDE_GPS=0
    GPS_HISTORY_MAX_RECORDS=0
    REMOTE_GPS_HISTORY=0
    ENABLE_ADVERT_ON_BOOT=1
    ENABLE_PRIVATE_KEY_IMPORT=1
    ENABLE_PRIVATE_KEY_EXPORT=1
    MESH_DEBUG=0
    MESH_PACKET_LOGGING=0
    ADVERT_NAME=\"TinyTab\"
    FIRMWARE_VERSION=\"v1.16.0-p4\"
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-parameter
    -Wno-sign-compare
    -Wno-error=reorder
    -Wno-error=format
    -Wno-error=class-memaccess
)
