cmake_minimum_required(VERSION 2.8.3)

find_package(Protobuf REQUIRED)

set(proto_dir ${CMAKE_CURRENT_SOURCE_DIR})

set(proto_files 
    ${proto_dir}/grSim_Commands.proto
    ${proto_dir}/grSim_Packet.proto
    ${proto_dir}/grSim_Replacement.proto
    )

foreach(proto_file ${proto_files})
    get_filename_component(file_name ${proto_file} NAME_WE)

    add_custom_target(${file_name}_target ALL COMMAND 
        "${PROTOBUF_PROTOC_EXECUTABLE}"
        "--proto_path=${proto_dir}" 
        "--python_out=${proto_dir}" 
        "${proto_file}"
    )
endforeach()
