# generated from genmsg/cmake/pkg-msg-paths.cmake.em

# message include dirs in installspace
_prepend_path("${navmap_DIR}/.." "msg" navmap_MSG_INCLUDE_DIRS UNIQUE)
set(navmap_MSG_DEPENDENCIES std_msgs;geometry_msgs)
