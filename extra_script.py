Import("env")

env.AddCustomTarget(
    name="ota",
    dependencies=None,
    always_build=True,
    actions=[
        "mkdir -p $BUILD_DIR/ota",
        "cp $BUILD_DIR/firmware.bin $BUILD_DIR/ota/firmware.bin",
        "cp $BUILD_DIR/bootloader.bin $BUILD_DIR/ota/bootloader.bin",
        "cp $BUILD_DIR/partitions.bin $BUILD_DIR/ota/partitions.bin"
    ]
)