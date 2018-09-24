#Get a list of all the cpp and header files and run clang-format on them.
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)
add_custom_target(
  clangformat
  COMMAND /usr/bin/clang-format-3.8
  -style=file
  -i
  ${ALL_SOURCE_FILES}
)
