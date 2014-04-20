# - Try to find PMD headers and plugins
# Once done this will define
#
#  PMD_INCLUDE_DIR - if PMD Header Include Directory is found
#
# The configuration directory PMD_PLUGIN_DIR will always be
# set, either to a directory containing the "digicam.L32.pap" file,
# or if no such file is found, to the local folder ".". In the last
# case a warning message is  issued.
#

FIND_PATH(PMD_INCLUDE_DIR NAMES pmdsdk2.h
  PATHS
  $ENV{PMDDIR}/include/
  ENV CPATH
  /usr/include/PMDSDK/include/
  /usr/local/include/PMDSDK/include/
  /opt/local/include/PMDSDK/include/
  NO_DEFAULT_PATH
)

FIND_PATH(PMD_PLUGIN_DIR NAMES camboardnano.L64.pap
  PATHS
  $ENV{PMDDIR}/bin/
  /usr/include/PMDSDK/bin/
  /usr/local/include/PMDSDK/bin/
  /opt/local/include/PMDSDK/bin/
  NO_DEFAULT_PATH
)

IF (NOT PMD_PLUGIN_FOUND)
FIND_PATH(PMD_PLUGIN_DIR NAMES camboardnano.L32.pap
  PATHS
  $ENV{PMDDIR}/bin/
  /usr/include/PMDSDK/bin/
  /usr/local/include/PMDSDK/bin/
  /opt/local/include/PMDSDK/bin/
  NO_DEFAULT_PATH
)
ENDIF (NOT PMD_PLUGIN_FOUND)

FIND_LIBRARY(PMD_LIBRARY NAMES pmdaccess2
  PATHS
  $ENV{PMDDIR}/bin/
  /usr/include/PMDSDK/bin/
  /usr/local/include/PMDSDK/bin/
  /opt/local/include/PMDSDK/bin/
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(PMD DEFAULT_MSG PMD_INCLUDE_DIR PMD_PLUGIN_DIR PMD_LIBRARY)

# show warning that no include directory was found; set it to "./" by default
IF (NOT PMD_INCLUDE_DIR)
 MESSAGE(STATUS "Warning: pmd include directory not found. Please ckeck your installation.")
 SET(PMD_INCLUDE_DIR .)
ENDIF (NOT PMD_INCLUDE_DIR)

# show warning that no include directory was found; set it to "./" by default
IF (NOT PMD_PLUGIN_DIR)
 MESSAGE(STATUS "Warning: pmd plugins directory not found. Please ckeck your installation.")
 SET(PMD_PLUGIN_DIR .)
ENDIF (NOT PMD_PLUGIN_DIR)

IF (NOT PMD_LIBRARY)
 MESSAGE(STATUS "Warning: pmd library not found. Please ckeck your installation.")
 SET(PMD_PLUGIN_DIR .)
ENDIF (NOT PMD_LIBRARY)


