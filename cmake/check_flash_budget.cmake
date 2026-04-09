if(NOT DEFINED ELF_FILE OR NOT DEFINED SIZE_TOOL)
  message(FATAL_ERROR "check_flash_budget: ELF_FILE and SIZE_TOOL are required")
endif()

if(NOT DEFINED APP_FLASH_BYTES)
  set(APP_FLASH_BYTES 57344)
endif()

if(NOT DEFINED RELEASE_BUDGET_PERCENT)
  set(RELEASE_BUDGET_PERCENT 60)
endif()

if(NOT DEFINED DEBUG_BUDGET_PERCENT)
  set(DEBUG_BUDGET_PERCENT 75)
endif()

execute_process(
  COMMAND ${SIZE_TOOL} ${ELF_FILE}
  OUTPUT_VARIABLE SIZE_OUT
  RESULT_VARIABLE SIZE_RET
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT SIZE_RET EQUAL 0)
  if(DEFINED C_COMPILER_PATH)
    get_filename_component(_tool_dir "${C_COMPILER_PATH}" DIRECTORY)
    get_filename_component(_cc_name "${C_COMPILER_PATH}" NAME)
    string(REPLACE "gcc" "size" _size_name "${_cc_name}")
    set(_size_fallback "${_tool_dir}/${_size_name}")
    execute_process(
      COMMAND ${_size_fallback} ${ELF_FILE}
      OUTPUT_VARIABLE SIZE_OUT
      RESULT_VARIABLE SIZE_RET
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif()
endif()

if(NOT SIZE_RET EQUAL 0)
  message(FATAL_ERROR "check_flash_budget: failed to run size tool (${SIZE_TOOL})")
endif()

string(REGEX MATCH "\n[ \t]*([0-9]+)[ \t]+([0-9]+)[ \t]+([0-9]+)[ \t]+([0-9]+)[ \t]+([0-9A-Fa-f]+)[ \t]+[^ \n]+" SIZE_LINE "${SIZE_OUT}")
if(NOT SIZE_LINE)
  message(WARNING "check_flash_budget: failed to parse size output, skip budget check")
  return()
endif()

set(TEXT_SIZE "${CMAKE_MATCH_1}")
set(DATA_SIZE "${CMAKE_MATCH_2}")
math(EXPR FLASH_USED "${TEXT_SIZE} + ${DATA_SIZE}")
math(EXPR FLASH_USED_PERMILLE "((${FLASH_USED} * 1000) + (${APP_FLASH_BYTES} / 2)) / ${APP_FLASH_BYTES}")
math(EXPR FLASH_USED_INT "${FLASH_USED_PERMILLE} / 10")
math(EXPR FLASH_USED_DEC "${FLASH_USED_PERMILLE} % 10")

if(NOT DEFINED BUILD_CFG)
  set(BUILD_CFG "")
endif()

message(STATUS
  "FLASH budget check (${BUILD_CFG}): used=${FLASH_USED}/${APP_FLASH_BYTES} bytes (${FLASH_USED_INT}.${FLASH_USED_DEC}%)"
)

if(BUILD_CFG STREQUAL "Release")
  math(EXPR RELEASE_BUDGET_BYTES "(${APP_FLASH_BYTES} * ${RELEASE_BUDGET_PERCENT}) / 100")
  if(FLASH_USED GREATER RELEASE_BUDGET_BYTES)
    message(FATAL_ERROR
      "Release FLASH budget exceeded: ${FLASH_USED} > ${RELEASE_BUDGET_BYTES} bytes (${RELEASE_BUDGET_PERCENT}%)"
    )
  endif()
elseif(BUILD_CFG STREQUAL "Debug")
  math(EXPR DEBUG_BUDGET_BYTES "(${APP_FLASH_BYTES} * ${DEBUG_BUDGET_PERCENT}) / 100")
  if(FLASH_USED GREATER DEBUG_BUDGET_BYTES)
    message(WARNING
      "Debug FLASH budget exceeded: ${FLASH_USED} > ${DEBUG_BUDGET_BYTES} bytes (${DEBUG_BUDGET_PERCENT}%)"
    )
  endif()
endif()
