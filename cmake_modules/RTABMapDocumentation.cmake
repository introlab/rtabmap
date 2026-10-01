# RTABMAP_SETUP_DOCUMENTATION()
#
# Prepares everything the API documentation build needs, and writes the Doxyfile:
#
#   - fetches Doxygen Awesome, the theme of the generated HTML
#   - regenerates the Doxygen HTML header and injects our scripts in it
#   - generates the "Parameter reference" page from Parameters.h
#   - configures Doxyfile.in into ${CMAKE_BINARY_DIR}/Doxyfile
#
# Call it only when the documentation is actually wanted (see BUILD_DOCUMENTATION
# in the top-level CMakeLists.txt): it needs the network, Doxygen and Python3.
# Run the result from the source tree, whose layout the Doxyfile paths assume:
#
#   doxygen <build dir>/Doxyfile
#
# The convenience script docs-report.sh does all of that, and lays the output out
# the way the published site expects.

MACRO(RTABMAP_SETUP_DOCUMENTATION)
   # Doxygen Awesome, the theme of the generated HTML. Downloaded here instead of
   # being vendored, so bumping it is a one-line change; the archive is pinned by
   # hash. It requires GENERATE_TREEVIEW=YES and HTML_COLORSTYLE=LIGHT (see
   # Doxyfile.in): the theme brings its own dark mode.
   INCLUDE(FetchContent)
   FetchContent_Declare(
      doxygen-awesome-css
      URL      https://github.com/jothepro/doxygen-awesome-css/archive/refs/tags/v2.3.4.zip
      URL_HASH SHA256=cb684b29f6be9e63300de56f26c13a04c298af8937a15ce87390fb5a601022a3
   )
   FetchContent_MakeAvailable(doxygen-awesome-css)
   FetchContent_GetProperties(doxygen-awesome-css SOURCE_DIR AWESOME_CSS_DIR)

   # Layout: the base theme keeps Doxygen's top tab bar. The sidebar-only variant
   # drops it and puts everything in the left navigation, which needs the wider
   # tree the theme expects (--side-nav-fixed-width).
   OPTION(BUILD_DOCUMENTATION_SIDEBAR "Use the sidebar-only layout of the Doxygen theme" OFF)
   SET(RTABMAP_DOXYGEN_STYLESHEETS "\"${AWESOME_CSS_DIR}/doxygen-awesome.css\"")
   SET(RTABMAP_DOXYGEN_TREEVIEW_WIDTH 250)
   IF(BUILD_DOCUMENTATION_SIDEBAR)
      SET(RTABMAP_DOXYGEN_TREEVIEW_WIDTH 335)
      STRING(APPEND RTABMAP_DOXYGEN_STYLESHEETS
         " \\\n                         \"${AWESOME_CSS_DIR}/doxygen-awesome-sidebar-only.css\""
         " \\\n                         \"${AWESOME_CSS_DIR}/doxygen-awesome-sidebar-only-darkmode-toggle.css\"")
   ENDIF()
   # Ours, last so it can override the theme's variables.
   STRING(APPEND RTABMAP_DOXYGEN_STYLESHEETS
      " \\\n                         \"${PROJECT_SOURCE_DIR}/doxygen/custom.css\"")

   SET(RTABMAP_DOXYGEN_AWESOME_TOGGLE ${AWESOME_CSS_DIR}/doxygen-awesome-darkmode-toggle.js)
   # Loaded from the header below; init() runs in <head> so that a reader who
   # picked dark mode never sees a flash of the light theme.
   SET(RTABMAP_DOXYGEN_AWESOME_SCRIPTS
      "<script type=\"text/javascript\" src=\"$relpath^doxygen-awesome-darkmode-toggle.js\"></script>\n<script type=\"text/javascript\">DoxygenAwesomeDarkModeToggle.init();</script>\n")

   # Doxygen HTML header carrying the version-switcher and Doxygen Awesome scripts.
   # The template is regenerated with `doxygen -w` so it always matches the locally
   # installed Doxygen (a committed header goes stale and warns on every run); we
   # only insert <script> tags before </head>. $relpath^ is Doxygen's path from the
   # current page back to this build's root, which is how the switcher locates
   # versions.js one level above (the API root) without knowing the site prefix.
   # The dark-mode toggle must be initialized before the page is rendered, so that
   # a reader who picked dark does not get a flash of the light theme.
   FIND_PROGRAM(RTABMAP_DOXYGEN_EXECUTABLE doxygen)
   SET(RTABMAP_DOXYGEN_HEADER "")
   IF(RTABMAP_DOXYGEN_EXECUTABLE)
      SET(RTABMAP_DOXYGEN_HEADER ${PROJECT_BINARY_DIR}/doxygen-header.html)
      EXECUTE_PROCESS(
         COMMAND ${RTABMAP_DOXYGEN_EXECUTABLE} -w html
                 ${RTABMAP_DOXYGEN_HEADER}
                 ${PROJECT_BINARY_DIR}/doxygen-footer.html
                 ${PROJECT_BINARY_DIR}/doxygen-stylesheet.css
         WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
         OUTPUT_QUIET ERROR_QUIET)
      IF(EXISTS ${RTABMAP_DOXYGEN_HEADER})
         FILE(READ ${RTABMAP_DOXYGEN_HEADER} RTABMAP_DOXYGEN_HEADER_CONTENT)
         STRING(REPLACE "</head>"
            "<script type=\"text/javascript\">window.RTABMAP_DOC_ROOT=\"$relpath^\";</script>\n<script type=\"text/javascript\" src=\"$relpath^../versions.js\"></script>\n<script type=\"text/javascript\" src=\"$relpath^version-switcher.js\"></script>\n${RTABMAP_DOXYGEN_AWESOME_SCRIPTS}</head>"
            RTABMAP_DOXYGEN_HEADER_CONTENT "${RTABMAP_DOXYGEN_HEADER_CONTENT}")
         FILE(WRITE ${RTABMAP_DOXYGEN_HEADER} "${RTABMAP_DOXYGEN_HEADER_CONTENT}")
      ELSE()
         SET(RTABMAP_DOXYGEN_HEADER "")
      ENDIF()
   ENDIF()

   # "Parameter reference" page: the parameters are declared through the
   # RTABMAP_PARAM* macros, so Doxygen alone can only show 1900 bare accessors.
   # The generator reads the declarations and emits one table per group, with the
   # key, type, default and description together. Optional: without Python the
   # page is simply left out of INPUT.
   FIND_PACKAGE(Python3 COMPONENTS Interpreter QUIET)
   SET(RTABMAP_DOXYGEN_PARAMETERS_PAGE "")
   IF(Python3_Interpreter_FOUND)
      EXECUTE_PROCESS(
         COMMAND ${Python3_EXECUTABLE}
                 ${PROJECT_SOURCE_DIR}/doxygen/generate_parameters_page.py
                 --input ${PROJECT_SOURCE_DIR}/corelib/include/${PROJECT_PREFIX}/core/Parameters.h
                 --output ${PROJECT_BINARY_DIR}/doxygen/parameters.md
         RESULT_VARIABLE RTABMAP_DOXYGEN_PARAMETERS_RESULT
         OUTPUT_QUIET ERROR_QUIET)
      IF(RTABMAP_DOXYGEN_PARAMETERS_RESULT EQUAL 0)
         SET(RTABMAP_DOXYGEN_PARAMETERS_PAGE ${PROJECT_BINARY_DIR}/doxygen/parameters.md)
      ELSE()
         MESSAGE(WARNING "Could not generate the Doxygen parameter reference page.")
      ENDIF()
   ENDIF()

   # Doxyfile with PROJECT_NUMBER filled from RTABMAP_VERSION, so the generated
   # documentation can never drift from the actual version. Its INPUT/OUTPUT paths
   # are relative to the source tree, so run it from there:
   #   doxygen <build dir>/Doxyfile
   CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile @ONLY)
ENDMACRO(RTABMAP_SETUP_DOCUMENTATION)
