AUX_SOURCE_DIRECTORY(. CPP_FILES)

add_library( studyslam SHARED ${CPP_FILES})

target_link_libraries( studyslam
    ${THIRD_PARTY_LIBS}
)