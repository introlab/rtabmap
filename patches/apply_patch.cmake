# Try to apply the patch in reverse first to see if it's already there
execute_process(
    COMMAND git apply --reverse --check "${PATCH_FILE}"
    RESULT_VARIABLE PATCH_APPLIED
    OUTPUT_QUIET
    ERROR_QUIET
)

if(NOT PATCH_APPLIED EQUAL 0)
    # If not applied, apply it now with whitespace fixes for Windows
    execute_process(
        COMMAND git apply --ignore-whitespace --whitespace=fix "${PATCH_FILE}"
        RESULT_VARIABLE RESULT
    )
    if(NOT RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to apply patch: ${PATCH_FILE}")
    endif()
endif()