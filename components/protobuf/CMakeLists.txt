# Nanopb Protobuf Component for ESP-IDF
# Uses pre-generated protobuf C files (generated from imu_sample.proto)
#
# To regenerate after proto changes:
#   pip install protobuf grpcio-tools
#   git clone --depth 1 --branch 0.4.9 https://github.com/nanopb/nanopb.git
#   python3 nanopb/generator/nanopb_generator.py imu_sample.proto
#   rm -rf nanopb

idf_component_register(
    SRCS 
        "pb_common.c"
        "pb_encode.c"
        "pb_decode.c"
        "imu_sample.pb.c"
        "protobuf_utils.c"
    INCLUDE_DIRS 
        "."
        "include"
    REQUIRES
        imu_bno055
)
