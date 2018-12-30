# Find package sqlite3 (headers and library)
#
# Export:
# SQLITE3_INCLUDE_DIRS
# SQLITE3_LIBRARIES
# SQLITE3_FOUND

FIND_PATH(SQLITE3_INCLUDE_DIR sqlite3.h
  /usr/include
  /usr/local/include
  )
FIND_LIBRARY(SQLITE3_LIBRARY
  NAMES sqlite3
  PATHS /usr/lib /usr/local/lib /usr/lib64 /usr/local/lib64
  )

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(SQLITE3 DEFAULT_MSG SQLITE3_LIBRARY SQLITE3_INCLUDE_DIR)

IF(SQLITE3_FOUND)
	SET(SQLITE3_LIBRARIES ${SQLITE3_LIBRARY})
	SET(SQLITE3_INCLUDE_DIRS ${SQLITE3_INCLUDE_DIR})
ELSE(SQLITE3_FOUND)
	SET(SQLITE3_LIBRARIES)
	SET(SQLITE3_INCLUDE_DIRS)
ENDIF(SQLITE3_FOUND)

MARK_AS_ADVANCED(SQLITE3_INCLUDE_DIRS SQLITE3_LIBRARIES)