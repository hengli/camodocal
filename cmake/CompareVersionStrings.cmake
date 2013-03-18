# Computes the relationship between two version strings.  A version
# string is a number delineated by '.'s such as 1.3.2 and 0.99.9.1.
# You can feed version strings with different number of dot versions,
# and the shorter version number will be padded with zeros: 9.2 <
# 9.2.1 will actually compare 9.2.0 < 9.2.1.
#
# Input: a_in - value, not variable
#        b_in - value, not variable
#        result_out - variable with value:
#                         -1 : a_in <  b_in
#                          0 : a_in == b_in
#                          1 : a_in >  b_in
#        optional argument - TRUE or default FALSE:
#              When passing 1.2.3 and 1.2 and this switch is on
#              the function will actually compare 1.2 < 1.2
#              Caution: Only the first string is adjusted!
#
# Written by James Bigler.
# Extended with an optional argument by Reto Grieder

FUNCTION(COMPARE_VERSION_STRINGS a_in b_in result_out)
  # Additional argument can be a switch to change compare behaviour
  SET(cut_first ${ARGN})

  # Replace '.' with ';' to allow easy tokenization of the string.
  STRING(REPLACE "." ";" a ${a_in})
  STRING(REPLACE "." ";" b ${b_in})

  # Check the size of each list to see if they are equal.
  LIST(LENGTH a a_length)
  LIST(LENGTH b b_length)

  # Pad the shorter list with zeros.

  IF(a_length LESS b_length)
    # a is shorter
    SET(shorter a)
    MATH(EXPR pad_range "${b_length} - ${a_length} - 1")
  ELSE(a_length LESS b_length)
    # b is shorter
    SET(shorter b)
    SET(first_longer a)
    MATH(EXPR pad_range "${a_length} - ${b_length} - 1")
  ENDIF(a_length LESS b_length)

  # PAD out if we need to
  IF(NOT pad_range LESS 0)
    FOREACH(pad RANGE ${pad_range})
      # Since shorter is an alias for b, we need to get to it by dereferencing shorter.
      IF(cut_first AND first_longer)
        LIST(REMOVE_AT a -1) # remove last element
      ELSE(cut_first AND first_longer)
        LIST(APPEND ${shorter} 0)
      ENDIF(cut_first AND first_longer)
    ENDFOREACH(pad)
  ENDIF(NOT pad_range LESS 0)

  SET(result 0)
  SET(index 0)
  FOREACH(a_version ${a})
    IF(result EQUAL 0)
      # Only continue to compare things as long as they are equal
      LIST(GET b ${index} b_version)
      # LESS
      IF(a_version LESS b_version)
        SET(result -1)
      ENDIF(a_version LESS b_version)
      # GREATER
      IF(a_version GREATER b_version)
        SET(result 1)
      ENDIF(a_version GREATER b_version)
    ENDIF(result EQUAL 0)
    MATH(EXPR index "${index} + 1")
  ENDFOREACH(a_version)

  # Copy out the return result
  SET(${result_out} ${result} PARENT_SCOPE)
ENDFUNCTION(COMPARE_VERSION_STRINGS)
